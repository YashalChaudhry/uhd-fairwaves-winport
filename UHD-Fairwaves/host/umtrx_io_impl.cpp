#include "umtrx_impl.hpp"
#include "umtrx_regs.hpp"
#include "usrp2/fw_common.h"
#include "cores/validate_subdev_spec.hpp"
#include "cores/async_packet_handler.hpp"
#include "cores/super_recv_packet_handler.hpp"
#include "cores/super_send_packet_handler.hpp"
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/log.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#ifdef THREAD_PRIORITY_HPP_DEPRECATED
#  include <uhd/utils/thread.hpp>
#else
#  include <uhd/utils/thread_priority.hpp>
#endif

static const size_t DEFAULT_NUM_FRAMES = 32;

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace pt = boost::posix_time;

static UHD_INLINE pt::time_duration to_time_dur(double timeout) {
    return pt::microseconds(long(timeout * 1e6));
}

static const size_t vrt_send_header_offset_words32 = 1;

// Custom concrete implementations - MUST be defined before use
class umtrx_recv_packet_streamer : public sph::recv_packet_streamer {
public:
    umtrx_recv_packet_streamer(size_t max_num_samps) : recv_packet_streamer(max_num_samps) {}

    // Implement the pure virtual methods from rx_streamer
    void post_input_action(const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t) override {
        // UmTRX doesn't use RFNoC actions, so this can be empty
    }
};

class umtrx_send_packet_streamer : public sph::send_packet_streamer {
public:
    umtrx_send_packet_streamer(size_t max_num_samps) : send_packet_streamer(max_num_samps) {}

    // Implement the pure virtual methods from tx_streamer
    void post_output_action(const std::shared_ptr<uhd::rfnoc::action_info>&, const size_t) override {
        // UmTRX doesn't use RFNoC actions, so this can be empty
    }
};

void umtrx_impl::update_rx_subdev_spec(const uhd::usrp::subdev_spec_t& spec)
{
    validate_subdev_spec(_tree, spec, "rx");
    boost::uint32_t rx_fe_sw = 0;
    for (size_t i = 0; i < spec.size(); i++)
    {
        UHD_ASSERT_THROW(spec[i].sd_name == "0");
        UHD_ASSERT_THROW(spec[i].db_name == "A" or spec[i].db_name == "B");
        if (spec[i].db_name == "A") rx_fe_sw |= (0 << i);
        if (spec[i].db_name == "B") rx_fe_sw |= (1 << i);
    }
    _ctrl->poke32(U2_REG_SR_ADDR(SR_RX_FE_SW), rx_fe_sw);
}

void umtrx_impl::update_tx_subdev_spec(const uhd::usrp::subdev_spec_t& spec)
{
    validate_subdev_spec(_tree, spec, "tx");
    boost::uint32_t tx_fe_sw = 0;
    for (size_t i = 0; i < spec.size(); i++)
    {
        UHD_ASSERT_THROW(spec[i].sd_name == "0");
        UHD_ASSERT_THROW(spec[i].db_name == "A" or spec[i].db_name == "B");
        if (i == 1) UHD_ASSERT_THROW(spec[0].db_name != spec[1].db_name);
        if (i == 0) tx_fe_sw = (spec[i].db_name == "A") ? 0 : 1;
    }
    _ctrl->poke32(U2_REG_SR_ADDR(SR_TX_FE_SW), tx_fe_sw);
}

void umtrx_impl::update_rates(void)
{
    fs_path root = "/mboards/0";
    _tree->access<double>(root / "tick_rate").update();

    BOOST_FOREACH(const std::string & name, _tree->list(root / "rx_dsps"))
    {
        _tree->access<double>(root / "rx_dsps" / name / "rate" / "value").update();
    }
    BOOST_FOREACH(const std::string & name, _tree->list(root / "tx_dsps"))
    {
        _tree->access<double>(root / "tx_dsps" / name / "rate" / "value").update();
    }
}

void umtrx_impl::update_rx_samp_rate(const size_t dsp, const double rate)
{
    // Use concrete class for dynamic_pointer_cast
    UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_recv_packet_streamer> my_streamer =
        UMTRX_UHD_PTR_NAMESPACE::dynamic_pointer_cast<umtrx_recv_packet_streamer>(_rx_streamers[dsp].lock());
    if (not my_streamer) return;

    my_streamer->set_samp_rate(rate);
    const double adj = _rx_dsps[dsp]->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

void umtrx_impl::update_tx_samp_rate(const size_t dsp, const double rate)
{
    // Use concrete class for dynamic_pointer_cast
    UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_send_packet_streamer> my_streamer =
        UMTRX_UHD_PTR_NAMESPACE::dynamic_pointer_cast<umtrx_send_packet_streamer>(_tx_streamers[dsp].lock());
    if (not my_streamer) return;

    my_streamer->set_samp_rate(rate);
    const double adj = _tx_dsps[dsp]->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

void umtrx_impl::update_tick_rate(const double rate)
{
    for (size_t i = 0; i < _rx_streamers.size(); i++)
    {
        // Use concrete class for dynamic_pointer_cast
        UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_recv_packet_streamer> my_streamer =
            UMTRX_UHD_PTR_NAMESPACE::dynamic_pointer_cast<umtrx_recv_packet_streamer>(_rx_streamers[i].lock());
        if (not my_streamer) continue;
        my_streamer->set_tick_rate(rate);
    }
    for (size_t i = 0; i < _tx_streamers.size(); i++)
    {
        // Use concrete class for dynamic_pointer_cast
        UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_send_packet_streamer> my_streamer =
            UMTRX_UHD_PTR_NAMESPACE::dynamic_pointer_cast<umtrx_send_packet_streamer>(_tx_streamers[i].lock());
        if (not my_streamer) continue;
        my_streamer->set_tick_rate(rate);
    }
}

static void program_stream_dest(zero_copy_if::sptr& xport, const size_t which)
{
    while (xport->get_recv_buff(0.0)) {}

    usrp2_stream_ctrl_t stream_ctrl = usrp2_stream_ctrl_t();
    stream_ctrl.sequence = uhd::htonx(boost::uint32_t(0));
    stream_ctrl.vrt_hdr = uhd::htonx(boost::uint32_t(USRP2_INVALID_VRT_HEADER));
    stream_ctrl.which = uhd::htonx(boost::uint32_t(which));

    managed_send_buffer::sptr send_buff = xport->get_send_buff();
    std::memcpy(send_buff->cast<void*>(), &stream_ctrl, sizeof(stream_ctrl));
    send_buff->commit(sizeof(stream_ctrl));
}

uhd::transport::zero_copy_if::sptr umtrx_impl::make_xport(const size_t which, const uhd::device_addr_t& args)
{
    zero_copy_xport_params default_params;
    default_params.send_frame_size = transport::udp_simple::mtu;
    default_params.recv_frame_size = transport::udp_simple::mtu;
    default_params.num_send_frames = DEFAULT_NUM_FRAMES;
    default_params.num_recv_frames = DEFAULT_NUM_FRAMES;
    udp_zero_copy::buff_params ignored_params;
    zero_copy_if::sptr xport = udp_zero_copy::make(_device_ip_addr, BOOST_STRINGIZE(USRP2_UDP_SERVER_PORT), default_params, ignored_params, args);
    program_stream_dest(xport, which);
    _iface->peek32(0);
    return xport;
}

uhd::rx_streamer::sptr umtrx_impl::get_rx_stream(const uhd::stream_args_t& args_)
{
    boost::mutex::scoped_lock l(_setupMutex);
    stream_args_t args = args_;

    args.otw_format = args.otw_format.empty() ? "sc16" : args.otw_format;
    args.channels = args.channels.empty() ? std::vector<size_t>(1, 0) : args.channels;

    if (not args.args.has_key("recv_buff_size"))
    {
#if defined(UHD_PLATFORM_MACOS) || defined(UHD_PLATFORM_BSD)
        args.args["recv_buff_size"] = "1e6";
#elif defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)
        args.args["recv_buff_size"] = "50e6";
#endif
    }

    std::vector<zero_copy_if::sptr> xports;
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++)
    {
        const size_t dsp = args.channels[chan_i];
        size_t which = ~0;
        if (dsp == 0) which = UMTRX_DSP_RX0_FRAMER;
        if (dsp == 1) which = UMTRX_DSP_RX1_FRAMER;
        if (dsp == 2) which = UMTRX_DSP_RX2_FRAMER;
        if (dsp == 3) which = UMTRX_DSP_RX3_FRAMER;
        UHD_ASSERT_THROW(which != size_t(~0));
        xports.push_back(make_xport(which, args.args));
    }

    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32 * sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr)
        - sizeof(vrt::if_packet_info_t().cid)
        - sizeof(vrt::if_packet_info_t().tsi)
        ;
    const size_t bpp = xports[0]->get_recv_frame_size() - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp / bpi));

    // Use concrete class for make_shared
    UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_recv_packet_streamer> my_streamer =
        UMTRX_UHD_PTR_NAMESPACE::make_shared<umtrx_recv_packet_streamer>(spp);

    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_unpacker(&vrt::if_hdr_unpack_be);

    uhd::convert::id_type id;
    id.input_format = args.otw_format + "_item32_be";
    id.num_inputs = 1;
    id.output_format = args.cpu_format;
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++)
    {
        const size_t dsp = args.channels[chan_i];
        _rx_dsps[dsp]->set_nsamps_per_packet(spp);
        _rx_dsps[dsp]->setup(args);
        my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
            &zero_copy_if::get_recv_buff, xports[chan_i], boost::placeholders::_1
        ), true);
        my_streamer->set_issue_stream_cmd(chan_i, boost::bind(
            &rx_dsp_core_200::issue_stream_command, _rx_dsps[dsp], boost::placeholders::_1));
        _rx_streamers[dsp] = my_streamer;
    }

    const size_t packets_per_sock_buff = size_t(50e6 / xports[0]->get_recv_frame_size());
    my_streamer->set_alignment_failure_threshold(packets_per_sock_buff);

    this->update_rates();

    return my_streamer;
}

class flow_control_monitor {
public:
    typedef boost::uint32_t seq_type;
    typedef boost::shared_ptr<flow_control_monitor> sptr;

    flow_control_monitor(seq_type max_seqs_out) :_max_seqs_out(max_seqs_out) {
        this->clear();
        _ready_fcn = boost::bind(&flow_control_monitor::ready, this);
    }

    void clear(void) {
        _last_seq_out = 0;
        _last_seq_ack = 0;
    }

    UHD_INLINE seq_type get_curr_seq_out(void) {
        return _last_seq_out++;
    }

    UHD_INLINE bool check_fc_condition(double timeout) {
        boost::mutex::scoped_lock lock(_fc_mutex);
        if (this->ready()) return true;
        boost::this_thread::disable_interruption di;
        return _fc_cond.timed_wait(lock, to_time_dur(timeout), _ready_fcn);
    }

    UHD_INLINE void update_fc_condition(seq_type seq) {
        boost::mutex::scoped_lock lock(_fc_mutex);
        _last_seq_ack = seq;
        lock.unlock();
        _fc_cond.notify_one();
    }

private:
    bool ready(void) {
        return seq_type(_last_seq_out - _last_seq_ack) < _max_seqs_out;
    }

    boost::mutex _fc_mutex;
    boost::condition_variable _fc_cond;
    seq_type _last_seq_out, _last_seq_ack;
    const seq_type _max_seqs_out;
    boost::function<bool(void)> _ready_fcn;
};

static managed_send_buffer::sptr get_send_buff(
    task::sptr,
    flow_control_monitor::sptr fc_mon,
    zero_copy_if::sptr xport,
    double timeout
)
{
    if (not fc_mon->check_fc_condition(timeout)) return managed_send_buffer::sptr();

    managed_send_buffer::sptr buff = xport->get_send_buff(timeout);

    if (buff.get()) buff->cast<boost::uint32_t*>()[0] = uhd::htonx(fc_mon->get_curr_seq_out());

    return buff;
}

static void handle_tx_async_msgs(
    const size_t chan,
    const double tick_rate,
    flow_control_monitor::sptr fc_mon,
    zero_copy_if::sptr xport,
    boost::function<void(void)> stop_flow_control,
    boost::shared_ptr<umtrx_impl::async_md_type> async_queue,
    boost::shared_ptr<umtrx_impl::async_md_type> old_async_queue
) {
    set_thread_priority_safe();

    while (not boost::this_thread::interruption_requested())
    {
        managed_recv_buffer::sptr buff = xport->get_recv_buff();
        if (not buff) continue;

        try {
            vrt::if_packet_info_t if_packet_info;
            if_packet_info.num_packet_words32 = buff->size() / sizeof(boost::uint32_t);
            const boost::uint32_t* vrt_hdr = buff->cast<const boost::uint32_t*>();
            vrt::if_hdr_unpack_be(vrt_hdr, if_packet_info);

            if (if_packet_info.packet_type != vrt::if_packet_info_t::PACKET_TYPE_DATA)
            {
                async_metadata_t metadata;
                load_metadata_from_buff(uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, chan);

                if (metadata.event_code == 0) {
                    boost::uint32_t fc_word32 = (vrt_hdr + if_packet_info.num_header_words32)[1];
                    fc_mon->update_fc_condition(uhd::ntohx(fc_word32));
                    continue;
                }
                async_queue->push_with_pop_on_full(metadata);
                old_async_queue->push_with_pop_on_full(metadata);

                standard_async_msg_prints(metadata);
            }
        }
        catch (const std::exception& e) {
            UHD_LOGGER_ERROR("UMTRX") << "Error in handle_tx_async_msgs: " << e.what();
        }
    }

    stop_flow_control();
    size_t i = 0;
    while (not xport->get_recv_buff(0.01))
    {
        if (i++ > 100) break;
    }
}

uhd::tx_streamer::sptr umtrx_impl::get_tx_stream(const uhd::stream_args_t& args_)
{
    boost::mutex::scoped_lock l(_setupMutex);
    stream_args_t args = args_;

    args.otw_format = args.otw_format.empty() ? "sc16" : args.otw_format;
    args.channels = args.channels.empty() ? std::vector<size_t>(1, 0) : args.channels;

    if (not args.args.has_key("send_buff_size"))
    {
        args.args["send_buff_size"] = boost::lexical_cast<std::string>(UMTRX_SRAM_BYTES);
    }

    std::vector<zero_copy_if::sptr> xports;
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++)
    {
        const size_t dsp = args.channels[chan_i];
        size_t which = ~0;
        if (dsp == 0) which = UMTRX_DSP_TX0_FRAMER;
        if (dsp == 1) which = UMTRX_DSP_TX1_FRAMER;
        UHD_ASSERT_THROW(which != size_t(~0));
        xports.push_back(make_xport(which, args.args));
    }

    static const size_t hdr_size = 0
        + vrt_send_header_offset_words32 * sizeof(boost::uint32_t)
        + vrt::max_if_hdr_words32 * sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr)
        - sizeof(vrt::if_packet_info_t().cid)
        - sizeof(vrt::if_packet_info_t().tsi)
        ;
    const size_t bpp = xports[0]->get_send_frame_size() - hdr_size;
    const size_t spp = bpp / convert::get_bytes_per_item(args.otw_format);

    // Use concrete class for make_shared
    UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_send_packet_streamer> my_streamer =
        UMTRX_UHD_PTR_NAMESPACE::make_shared<umtrx_send_packet_streamer>(spp);

    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_packer(&vrt::if_hdr_pack_be, vrt_send_header_offset_words32);

    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = 1;
    id.output_format = args.otw_format + "_item32_be";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    boost::shared_ptr<async_md_type> async_md(new async_md_type(1000));
    if (not _old_async_queue) _old_async_queue.reset(new async_md_type(1000));
    my_streamer->set_async_receiver(boost::bind(&async_md_type::pop_with_timed_wait, async_md, boost::placeholders::_1, boost::placeholders::_2));

    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++)
    {
        const size_t dsp = args.channels[chan_i];
        _tx_dsps[dsp]->setup(args);

        boost::uint32_t sid = ~0;
        if (dsp == 0) sid = UMTRX_DSP_TX0_SID;
        if (dsp == 1) sid = UMTRX_DSP_TX1_SID;
        UHD_ASSERT_THROW(sid != boost::uint32_t(~0));
        my_streamer->set_xport_chan_sid(chan_i, true, sid);

        const size_t fc_window = UMTRX_SRAM_BYTES / xports[chan_i]->get_send_frame_size();
        flow_control_monitor::sptr fc_mon(new flow_control_monitor(fc_window));

        const double ups_per_sec = args.args.cast<double>("ups_per_sec", 20);
        const double ups_per_fifo = args.args.cast<double>("ups_per_fifo", 8.0);
        _tx_dsps[dsp]->set_updates(
            (ups_per_sec > 0.0) ? size_t(this->get_master_clock_rate() / ups_per_sec) : 0,
            (ups_per_fifo > 0.0) ? size_t(fc_window / ups_per_fifo) : 0
        );

        boost::function<void(void)> stop_flow_control = boost::bind(&tx_dsp_core_200::set_updates, _tx_dsps[dsp], 0, 0);
        task::sptr task = task::make(boost::bind(
            &handle_tx_async_msgs, chan_i, this->get_master_clock_rate(),
            fc_mon, xports[chan_i], stop_flow_control, async_md, _old_async_queue));

        my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
            &get_send_buff, task, fc_mon, xports[chan_i], boost::placeholders::_1
        ));

        _tx_streamers[dsp] = my_streamer;
    }

    this->update_rates();

    return my_streamer;
}

bool umtrx_impl::recv_async_msg(uhd::async_metadata_t& async_metadata, const double timeout)
{
    boost::this_thread::disable_interruption di;
    return _old_async_queue->pop_with_timed_wait(async_metadata, timeout);
}