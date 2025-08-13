#ifndef INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_HPP
#define INCLUDED_LIBUHD_TRANSPORT_SUPER_RECV_PACKET_HANDLER_HPP

#include <uhd/config.hpp>
#include <uhd/exception.hpp>
#include <uhd/convert.hpp>
#include <uhd/stream.hpp>
#include "umtrx_log_adapter.hpp"
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <uhd/utils/log.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/format.hpp>
#include <boost/bind/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/barrier.hpp>
#include <iostream>
#include <vector>

#ifdef UHD_TXRX_DEBUG_PRINTS
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#endif

namespace uhd {
    namespace transport {
        namespace sph {

            UHD_INLINE boost::uint32_t get_context_code(
                const boost::uint32_t* vrt_hdr, const vrt::if_packet_info_t& if_packet_info
            ) {
                boost::uint32_t word0 = vrt_hdr[if_packet_info.num_header_words32] |
                    uhd::byteswap(vrt_hdr[if_packet_info.num_header_words32]);
                return word0 & 0xff;
            }

            typedef boost::function<void(void)> handle_overflow_type;
            static inline void handle_overflow_nop(void) {}

            class recv_packet_handler {
            public:
                typedef boost::function<managed_recv_buffer::sptr(double)> get_buff_type;
                typedef boost::function<void(const size_t)> handle_flowctrl_type;
                typedef boost::function<void(const stream_cmd_t&)> issue_stream_cmd_type;
                typedef void(*vrt_unpacker_type)(const boost::uint32_t*, vrt::if_packet_info_t&);

                recv_packet_handler(const size_t size = 1) :
                    _queue_error_for_next_call(false),
                    _buffers_infos_index(0)
                {
#ifdef  ERROR_INJECT_DROPPED_PACKETS
                    recvd_packets = 0;
#endif

                    this->resize(size);
                    set_alignment_failure_threshold(1000);
                }

                ~recv_packet_handler(void) {
                }

                void resize(const size_t size) {
                    if (this->size() == size) return;
                    _props.resize(size);
                    _buffers_infos = std::vector<buffers_info_type>(4, buffers_info_type(size));
                }

                size_t size(void) const {
                    return _props.size();
                }

                void set_vrt_unpacker(const vrt_unpacker_type& vrt_unpacker, const size_t header_offset_words32 = 0) {
                    _vrt_unpacker = vrt_unpacker;
                    _header_offset_words32 = header_offset_words32;
                }

                void set_alignment_failure_threshold(const size_t threshold) {
                    _alignment_faulure_threshold = threshold * this->size();
                }

                void set_tick_rate(const double rate) {
                    _tick_rate = rate;
                }

                void set_samp_rate(const double rate) {
                    _samp_rate = rate;
                }

                void set_xport_chan_get_buff(const size_t xport_chan, const get_buff_type& get_buff, const bool flush = false) {
                    if (flush) {
                        while (get_buff(0.0));
                    }
                    _props.at(xport_chan).get_buff = get_buff;
                }

                void flush_all(const double timeout = 0.0)
                {
                    _flush_all(timeout);
                    return;
                }

                void set_xport_handle_flowctrl(const size_t xport_chan, const handle_flowctrl_type& handle_flowctrl, const size_t update_window, const bool do_init = false)
                {
                    _props.at(xport_chan).handle_flowctrl = handle_flowctrl;
                    _props.at(xport_chan).fc_update_window = std::min<size_t>(update_window, 0xfff);
                    if (do_init) handle_flowctrl(0);
                }

                void set_converter(const uhd::convert::id_type& id) {
                    _num_outputs = id.num_outputs;
                    _converter = uhd::convert::get_converter(id)();
                    this->set_scale_factor(1 / 32767.);
                    _bytes_per_otw_item = uhd::convert::get_bytes_per_item(id.input_format);
                    _bytes_per_cpu_item = uhd::convert::get_bytes_per_item(id.output_format);
                }

                void set_overflow_handler(const size_t xport_chan, const handle_overflow_type& handle_overflow) {
                    _props.at(xport_chan).handle_overflow = handle_overflow;
                }

                void set_scale_factor(const double scale_factor) {
                    _converter->set_scalar(scale_factor);
                }

                void set_issue_stream_cmd(const size_t xport_chan, const issue_stream_cmd_type& issue_stream_cmd)
                {
                    _props.at(xport_chan).issue_stream_cmd = issue_stream_cmd;
                }

                void issue_stream_cmd(const stream_cmd_t& stream_cmd)
                {
                    for (size_t i = 0; i < _props.size(); i++)
                    {
                        if (_props[i].issue_stream_cmd) _props[i].issue_stream_cmd(stream_cmd);
                    }
                }

                UHD_INLINE size_t recv(
                    const uhd::rx_streamer::buffs_type& buffs,
                    const size_t nsamps_per_buff,
                    uhd::rx_metadata_t& metadata,
                    const double timeout,
                    const bool one_packet
                ) {
                    if (_queue_error_for_next_call) {
                        _queue_error_for_next_call = false;
                        metadata = _queue_metadata;
                        if (_queue_metadata.error_code != rx_metadata_t::ERROR_CODE_TIMEOUT) return 0;
                    }

                    size_t accum_num_samps = recv_one_packet(
                        buffs, nsamps_per_buff, metadata, timeout
                    );

                    if (one_packet) {
#ifdef UHD_TXRX_DEBUG_PRINTS
                        dbg_gather_data(nsamps_per_buff, accum_num_samps, metadata, timeout, one_packet);
#endif
                        return accum_num_samps;
                    }

                    if (metadata.error_code != rx_metadata_t::ERROR_CODE_NONE) return accum_num_samps;

                    while (accum_num_samps < nsamps_per_buff) {
                        size_t num_samps = recv_one_packet(
                            buffs, nsamps_per_buff - accum_num_samps, _queue_metadata,
                            timeout, accum_num_samps * _bytes_per_cpu_item
                        );

                        if (_queue_metadata.error_code != rx_metadata_t::ERROR_CODE_NONE) {
                            _queue_error_for_next_call = true;
                            break;
                        }
                        accum_num_samps += num_samps;
                    }
#ifdef UHD_TXRX_DEBUG_PRINTS
                    dbg_gather_data(nsamps_per_buff, accum_num_samps, metadata, timeout, one_packet);
#endif
                    return accum_num_samps;
                }

            private:
                vrt_unpacker_type _vrt_unpacker;
                size_t _header_offset_words32;
                double _tick_rate, _samp_rate;
                bool _queue_error_for_next_call;
                size_t _alignment_faulure_threshold;
                rx_metadata_t _queue_metadata;
                struct xport_chan_props_type {
                    xport_chan_props_type(void) :
                        packet_count(0),
                        handle_overflow(&handle_overflow_nop),
                        fc_update_window(0)
                    {
                    }
                    get_buff_type get_buff;
                    issue_stream_cmd_type issue_stream_cmd;
                    size_t packet_count;
                    handle_overflow_type handle_overflow;
                    handle_flowctrl_type handle_flowctrl;
                    size_t fc_update_window;
                };
                std::vector<xport_chan_props_type> _props;
                size_t _num_outputs;
                size_t _bytes_per_otw_item;
                size_t _bytes_per_cpu_item;
                uhd::convert::converter::sptr _converter;

                struct per_buffer_info_type {
                    void reset()
                    {
                        buff.reset();
                        vrt_hdr = NULL;
                        time = time_spec_t(0.0);
                        copy_buff = NULL;
                    }
                    managed_recv_buffer::sptr buff;
                    const boost::uint32_t* vrt_hdr;
                    vrt::if_packet_info_t ifpi;
                    time_spec_t time;
                    const char* copy_buff;
                };

                struct buffers_info_type : std::vector<per_buffer_info_type> {
                    buffers_info_type(const size_t size) :
                        std::vector<per_buffer_info_type>(size),
                        indexes_todo(size, true),
                        alignment_time_valid(false),
                        data_bytes_to_copy(0),
                        fragment_offset_in_samps(0)
                    {
                    }
                    void reset()
                    {
                        indexes_todo.set();
                        alignment_time = time_spec_t(0.0);
                        alignment_time_valid = false;
                        data_bytes_to_copy = 0;
                        fragment_offset_in_samps = 0;
                        metadata.reset();
                        for (size_t i = 0; i < size(); i++)
                            at(i).reset();
                    }
                    boost::dynamic_bitset<> indexes_todo;
                    time_spec_t alignment_time;
                    bool alignment_time_valid;
                    size_t data_bytes_to_copy;
                    size_t fragment_offset_in_samps;
                    rx_metadata_t metadata;
                };

                std::vector<buffers_info_type> _buffers_infos;
                size_t _buffers_infos_index;
                buffers_info_type& get_curr_buffer_info(void) { return _buffers_infos[_buffers_infos_index]; }
                buffers_info_type& get_prev_buffer_info(void) { return _buffers_infos[(_buffers_infos_index + 3) % 4]; }
                buffers_info_type& get_next_buffer_info(void) { return _buffers_infos[(_buffers_infos_index + 1) % 4]; }
                void increment_buffer_info(void) { _buffers_infos_index = (_buffers_infos_index + 1) % 4; }

                enum packet_type {
                    PACKET_IF_DATA,
                    PACKET_TIMESTAMP_ERROR,
                    PACKET_INLINE_MESSAGE,
                    PACKET_TIMEOUT_ERROR,
                    PACKET_SEQUENCE_ERROR
                };

#ifdef  ERROR_INJECT_DROPPED_PACKETS
                int recvd_packets;
#endif

                UHD_INLINE packet_type get_and_process_single_packet(
                    const size_t index,
                    per_buffer_info_type& prev_buffer_info,
                    per_buffer_info_type& curr_buffer_info,
                    double timeout
                ) {
                    managed_recv_buffer::sptr& buff = curr_buffer_info.buff;
                    buff = _props[index].get_buff(timeout);
                    if (buff.get() == NULL) return PACKET_TIMEOUT_ERROR;

#ifdef  ERROR_INJECT_DROPPED_PACKETS
                    if (++recvd_packets > 1000)
                    {
                        recvd_packets = 0;
                        buff.reset();
                        buff = _props[index].get_buff(timeout);
                        if (buff.get() == NULL) return PACKET_TIMEOUT_ERROR;
                    }
#endif

                    size_t num_packet_words32 = buff->size() / sizeof(boost::uint32_t);
                    if (num_packet_words32 <= _header_offset_words32) {
                        throw std::runtime_error("recv buffer smaller than vrt packet offset");
                    }

                    per_buffer_info_type& info = curr_buffer_info;
                    info.ifpi.num_packet_words32 = num_packet_words32 - _header_offset_words32;
                    info.vrt_hdr = buff->cast<const boost::uint32_t*>() + _header_offset_words32;
                    _vrt_unpacker(info.vrt_hdr, info.ifpi);
                    info.time = time_spec_t::from_ticks(info.ifpi.tsf, _tick_rate);
                    info.copy_buff = reinterpret_cast<const char*>(info.vrt_hdr + info.ifpi.num_header_words32);

                    if (_props[index].handle_flowctrl)
                    {
                        if ((info.ifpi.packet_count % _props[index].fc_update_window) == 0)
                        {
                            _props[index].handle_flowctrl(info.ifpi.packet_count);
                        }
                    }

                    if (info.ifpi.packet_type != vrt::if_packet_info_t::PACKET_TYPE_DATA) {
                        return PACKET_INLINE_MESSAGE;
                    }

#ifndef SRPH_DONT_CHECK_SEQUENCE
                    const size_t seq_mask = (info.ifpi.link_type == vrt::if_packet_info_t::LINK_TYPE_NONE) ? 0xf : 0xfff;
                    const size_t expected_packet_count = _props[index].packet_count;
                    _props[index].packet_count = (info.ifpi.packet_count + 1) & seq_mask;
                    if (expected_packet_count != info.ifpi.packet_count) {
                        return PACKET_SEQUENCE_ERROR;
                    }
#endif

                    if (info.ifpi.has_tsf and prev_buffer_info.time > info.time) {
                        return PACKET_TIMESTAMP_ERROR;
                    }

                    return PACKET_IF_DATA;
                }

                void _flush_all(double timeout)
                {
                    for (size_t i = 0; i < _props.size(); i++)
                    {
                        per_buffer_info_type prev_buffer_info, curr_buffer_info;
                        while (true)
                        {
                            try
                            {
                                if (get_and_process_single_packet(
                                    i,
                                    prev_buffer_info,
                                    curr_buffer_info,
                                    timeout) == PACKET_TIMEOUT_ERROR) break;
                            }
                            catch (...) {}
                            prev_buffer_info = curr_buffer_info;
                            curr_buffer_info.reset();
                        }
                    }
                    get_prev_buffer_info().reset();
                    get_curr_buffer_info().reset();
                    get_next_buffer_info().reset();
                }

                UHD_INLINE void alignment_check(
                    const size_t index, buffers_info_type& info
                ) {
                    if (not info.alignment_time_valid or info[index].time > info.alignment_time) {
                        info.alignment_time_valid = true;
                        info.alignment_time = info[index].time;
                        info.indexes_todo.set();
                        info.indexes_todo.reset(index);
                        info.data_bytes_to_copy = info[index].ifpi.num_payload_bytes;
                    }
                    else if (info[index].time == info.alignment_time) {
                        info.indexes_todo.reset(index);
                    }
                }

                UHD_INLINE void get_aligned_buffs(double timeout) {

                    get_prev_buffer_info().reset();

                    increment_buffer_info();

                    buffers_info_type& prev_info = get_prev_buffer_info();
                    buffers_info_type& curr_info = get_curr_buffer_info();
                    buffers_info_type& next_info = get_next_buffer_info();

                    size_t iterations = 0;
                    while (curr_info.indexes_todo.any()) {

                        const size_t index = curr_info.indexes_todo.find_first();
                        packet_type packet;

                        try {
                            packet = get_and_process_single_packet(
                                index, prev_info[index], curr_info[index], timeout
                            );
                        }

                        catch (const std::exception& e) {
                            UHD_LOGGER_ERROR("UMTRX") << "The receive packet handler caught an exception: " << e.what();
                            std::swap(curr_info, next_info);
                            curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_BAD_PACKET;
                            return;
                        }

                        switch (packet) {
                        case PACKET_IF_DATA:
                            alignment_check(index, curr_info);
                            break;

                        case PACKET_TIMESTAMP_ERROR:
                            if (curr_info.alignment_time_valid and curr_info.alignment_time != curr_info[index].time) {
                                curr_info.alignment_time_valid = false;
                            }
                            alignment_check(index, curr_info);
                            break;

                        case PACKET_INLINE_MESSAGE:
                            std::swap(curr_info, next_info);
                            curr_info.metadata.has_time_spec = next_info[index].ifpi.has_tsf;
                            curr_info.metadata.time_spec = next_info[index].time;
                            curr_info.metadata.error_code = rx_metadata_t::error_code_t(get_context_code(next_info[index].vrt_hdr, next_info[index].ifpi));
                            if (curr_info.metadata.error_code == rx_metadata_t::ERROR_CODE_OVERFLOW) {
                                rx_metadata_t metadata = curr_info.metadata;
                                _props[index].handle_overflow();
                                curr_info.metadata = metadata;
                                UHD_LOG_FASTPATH("O");
                            }
                            return;

                        case PACKET_TIMEOUT_ERROR:
                            std::swap(curr_info, next_info);
                            curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
                            return;

                        case PACKET_SEQUENCE_ERROR:
                            alignment_check(index, curr_info);
                            std::swap(curr_info, next_info);
                            curr_info.metadata.has_time_spec = prev_info.metadata.has_time_spec;
                            curr_info.metadata.time_spec = prev_info.metadata.time_spec + time_spec_t::from_ticks(
                                prev_info[index].ifpi.num_payload_words32 * sizeof(boost::uint32_t) / _bytes_per_otw_item, _samp_rate);
                            curr_info.metadata.out_of_sequence = true;
                            curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
                            UHD_LOG_FASTPATH("D");
                            return;

                        }

                        if (iterations++ > _alignment_faulure_threshold) {
                            UHD_LOGGER_ERROR("UMTRX") << "The receive packet handler failed to time-align packets. "
                                << iterations << " received packets were processed by the handler. "
                                << "However, a timestamp match could not be determined.";
                            std::swap(curr_info, next_info);
                            curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_ALIGNMENT;
                            _props[index].handle_overflow();
                            return;
                        }

                    }

                    curr_info.metadata.has_time_spec = curr_info[0].ifpi.has_tsf;
                    curr_info.metadata.time_spec = curr_info[0].time;
                    curr_info.metadata.more_fragments = false;
                    curr_info.metadata.fragment_offset = 0;
                    curr_info.metadata.start_of_burst = curr_info[0].ifpi.sob;
                    curr_info.metadata.end_of_burst = curr_info[0].ifpi.eob;
                    curr_info.metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;

                }

                UHD_INLINE size_t recv_one_packet(
                    const uhd::rx_streamer::buffs_type& buffs,
                    const size_t nsamps_per_buff,
                    uhd::rx_metadata_t& metadata,
                    const double timeout,
                    const size_t buffer_offset_bytes = 0
                ) {
                    if (get_curr_buffer_info().data_bytes_to_copy == 0)
                    {
                        get_aligned_buffs(timeout);
                    }

                    buffers_info_type& info = get_curr_buffer_info();
                    metadata = info.metadata;

                    metadata.time_spec += time_spec_t::from_ticks(info.fragment_offset_in_samps, _samp_rate);

                    const size_t nsamps_available = info.data_bytes_to_copy / _bytes_per_otw_item;
                    const size_t nsamps_to_copy = std::min(nsamps_per_buff * _num_outputs, nsamps_available);
                    const size_t bytes_to_copy = nsamps_to_copy * _bytes_per_otw_item;
                    const size_t nsamps_to_copy_per_io_buff = nsamps_to_copy / _num_outputs;

                    _convert_nsamps = nsamps_to_copy_per_io_buff;
                    _convert_buffs = &buffs;
                    _convert_buffer_offset_bytes = buffer_offset_bytes;
                    _convert_bytes_to_copy = bytes_to_copy;

                    for (size_t i = 0; i < this->size(); i++) this->converter_thread_task(i);

                    info.data_bytes_to_copy -= bytes_to_copy;

                    metadata.more_fragments = info.data_bytes_to_copy != 0;
                    metadata.fragment_offset = info.fragment_offset_in_samps;
                    info.fragment_offset_in_samps += nsamps_to_copy;

                    return nsamps_to_copy_per_io_buff;
                }

                UHD_INLINE void converter_thread_task(const size_t index)
                {
                    buffers_info_type& buff_info = get_curr_buffer_info();
                    per_buffer_info_type& info = buff_info[index];
                    const rx_streamer::buffs_type& buffs = *_convert_buffs;

                    void* io_buffs[4];
                    for (size_t i = 0; i < _num_outputs; i++) {
                        char* b = reinterpret_cast<char*>(buffs[index * _num_outputs + i]);
                        io_buffs[i] = b + _convert_buffer_offset_bytes;
                    }
                    const ref_vector<void*> out_buffs(io_buffs, _num_outputs);

                    _converter->conv(info.copy_buff, out_buffs, _convert_nsamps);

                    info.copy_buff += _convert_bytes_to_copy;

                    if (buff_info.data_bytes_to_copy == _convert_bytes_to_copy) {
                        info.buff.reset();
                    }
                }

                size_t _convert_nsamps;
                const rx_streamer::buffs_type* _convert_buffs;
                size_t _convert_buffer_offset_bytes;
                size_t _convert_bytes_to_copy;

#ifdef UHD_TXRX_DEBUG_PRINTS
                struct dbg_recv_stat_t {
                    dbg_recv_stat_t(long wc, size_t nspb, size_t nsr, uhd::rx_metadata_t md, double to, bool op, double rate) :
                        wallclock(wc), nsamps_per_buff(nspb), nsamps_recv(nsr), metadata(md), timeout(to), one_packet(op), samp_rate(rate)
                    {
                    }
                    long wallclock;
                    size_t nsamps_per_buff;
                    size_t nsamps_recv;
                    uhd::rx_metadata_t metadata;
                    double timeout;
                    bool one_packet;
                    double samp_rate;
                    std::string print_line() {
                        boost::format fmt("recv,%ld,%f,%i,%i,%s,%i,%s,%s,%s,%i,%s,%ld");
                        fmt% wallclock;
                        fmt% timeout % (int)nsamps_per_buff % (int)nsamps_recv;
                        fmt % (one_packet ? "true" : "false");
                        fmt% metadata.error_code;
                        fmt % (metadata.start_of_burst ? "true" : "false") % (metadata.end_of_burst ? "true" : "false");
                        fmt % (metadata.more_fragments ? "true" : "false") % (int)metadata.fragment_offset;
                        fmt % (metadata.has_time_spec ? "true" : "false") % metadata.time_spec.to_ticks(samp_rate);
                        return fmt.str();
                    }
                };

                void dbg_gather_data(const size_t nsamps_per_buff, const size_t nsamps_recv,
                    uhd::rx_metadata_t& metadata, const double timeout,
                    const bool one_packet,
                    bool dbg_print_directly = true
                )
                {
                    dbg_recv_stat_t data(boost::get_system_time().time_of_day().total_microseconds(),
                        nsamps_per_buff,
                        nsamps_recv,
                        metadata,
                        timeout,
                        one_packet,
                        _samp_rate
                    );
                    if (dbg_print_directly) {
                        dbg_print_err(data.print_line());
                    }
                }

                void dbg_print_err(std::string msg) {
                    std::string dbg_prefix("super_recv_packet_handler,");
                    msg = dbg_prefix + msg;
                    fprintf(stderr, "%s\n", msg.c_str());
                }
#endif
            };

            class recv_packet_streamer : public recv_packet_handler, public rx_streamer {
            public:
                recv_packet_streamer(const size_t max_num_samps) {
                    _max_num_samps = max_num_samps;
                }

                size_t get_num_channels(void) const {
                    return this->size();
                }

                size_t get_max_num_samps(void) const {
                    return _max_num_samps;
                }

                size_t recv(
                    const rx_streamer::buffs_type& buffs,
                    const size_t nsamps_per_buff,
                    uhd::rx_metadata_t& metadata,
                    const double timeout,
                    const bool one_packet
                ) {
                    return recv_packet_handler::recv(buffs, nsamps_per_buff, metadata, timeout, one_packet);
                }

                void issue_stream_cmd(const stream_cmd_t& stream_cmd)
                {
                    return recv_packet_handler::issue_stream_cmd(stream_cmd);
                }

                // Pure virtual method implementations for modern UHD compatibility
                virtual std::string get_cpu_format(void) const {
                    return "fc32"; // Default CPU format, can be overridden if needed
                }

                virtual std::string get_otw_format(void) const {
                    return "sc16"; // Default over-the-wire format
                }

            private:
                size_t _max_num_samps;
            };

        }
    }
}

#endif