#include "umtrx_impl.hpp"
#include "umtrx_regs.hpp"
#include "umtrx_version.hpp"
#include "umtrx_log_adapter.hpp"
#include "cores/apply_corrections.hpp"
#include <uhd/utils/log.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/utility.hpp>
#include <boost/foreach.hpp>
#include <boost/version.hpp>

static int verbosity = 0;

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

const int umtrx_impl::UMTRX_VGA1_DEF = -20;
const int umtrx_impl::UMTRX_VGA2_DEF = 22;
const int umtrx_impl::UMTRX_VGA2_MIN = 0;

const double umtrx_impl::_dcdc_val_to_volt[umtrx_impl::DCDC_VER_COUNT][256] =
{
  {
     9.38,  9.38,  9.40,  9.42,  9.42,  9.44,  9.46,  9.46,  9.48,  9.50,
     9.50,  9.52,  9.54,  9.54,  9.56,  9.58,  9.58,  9.60,  9.60,  9.62,
     9.64,  9.66,  9.66,  9.68,  9.70,  9.70,  9.72,  9.74,  9.76,  9.76,
     9.78,  9.80,  9.82,  9.82,  9.84,  9.86,  9.88,  9.90,  9.92,  9.92,
     9.94,  9.96,  9.98,  9.98, 10.00, 10.02, 10.04, 10.06, 10.06, 10.08,
    10.10, 10.12, 10.14, 10.16, 10.18, 10.20, 10.20, 10.24, 10.24, 10.28,
    10.30, 10.32, 10.34, 10.34, 10.36, 10.38, 10.40, 10.42, 10.44, 10.46,
    10.48, 10.50, 10.52, 10.54, 10.56, 10.60, 10.62, 10.64, 10.66, 10.68,
    10.70, 10.72, 10.74, 10.76, 10.78, 10.80, 10.84, 10.86, 10.88, 10.90,
    10.94, 10.96, 10.98, 11.00, 11.02, 11.06, 11.06, 11.10, 11.12, 11.16,
    11.18, 11.20, 11.24, 11.26, 11.28, 11.32, 11.34, 11.38, 11.40, 11.44,
    11.46, 11.50, 11.50, 11.54, 11.58, 11.60, 11.64, 11.66, 11.70, 11.74,
    11.76, 11.80, 11.84, 11.86, 11.90, 11.94, 11.98, 12.00, 12.02, 12.06,
    12.10, 12.14, 12.18, 12.22, 12.26, 12.28, 12.32, 12.36, 12.40, 12.44,
    12.48, 12.54, 12.58, 12.62, 12.64, 12.68, 12.72, 12.76, 12.82, 12.86,
    12.90, 12.96, 13.00, 13.04, 13.10, 13.14, 13.20, 13.24, 13.30, 13.34,
    13.38, 13.44, 13.48, 13.54, 13.60, 13.66, 13.72, 13.76, 13.82, 13.88,
    13.94, 14.02, 14.06, 14.14, 14.20, 14.26, 14.30, 14.36, 14.42, 14.50,
    14.56, 14.64, 14.72, 14.78, 14.86, 14.92, 15.00, 15.08, 15.16, 15.24,
    15.32, 15.40, 15.46, 15.54, 15.62, 15.72, 15.80, 15.90, 16.00, 16.08,
    16.18, 16.28, 16.38, 16.48, 16.58, 16.68, 16.80, 16.90, 16.96, 17.08,
    17.20, 17.32, 17.44, 17.56, 17.68, 17.82, 17.94, 18.06, 18.20, 18.36,
    18.48, 18.64, 18.78, 18.94, 19.02, 19.18, 19.34, 19.50, 19.68, 19.84,
    20.02, 20.20, 20.38, 20.58, 20.76, 20.96, 21.18, 21.38, 21.60, 21.82,
    21.92, 22.16, 22.40, 22.66, 22.92, 23.18, 23.46, 23.74, 24.02, 24.30,
    24.62, 24.94, 25.28, 25.62, 25.98, 26.34
  },{
    4.84,  4.84,  4.86,  4.88,  4.88,  4.90,  4.92,  4.94,  4.94,  4.96,
    4.98,  5.00,  5.02,  5.02,  5.04,  5.06,  5.06,  5.08,  5.10,  5.12,
    5.12,  5.14,  5.16,  5.18,  5.20,  5.22,  5.22,  5.24,  5.26,  5.28,
    5.30,  5.32,  5.32,  5.34,  5.36,  5.38,  5.40,  5.42,  5.44,  5.46,
    5.48,  5.50,  5.50,  5.52,  5.54,  5.56,  5.58,  5.60,  5.62,  5.64,
    5.66,  5.68,  5.70,  5.72,  5.74,  5.76,  5.78,  5.80,  5.82,  5.86,
    5.88,  5.90,  5.92,  5.94,  5.96,  5.98,  6.00,  6.02,  6.04,  6.08,
    6.10,  6.12,  6.14,  6.16,  6.20,  6.22,  6.24,  6.28,  6.30,  6.32,
    6.34,  6.36,  6.40,  6.42,  6.44,  6.48,  6.50,  6.54,  6.56,  6.58,
    6.62,  6.64,  6.68,  6.70,  6.74,  6.76,  6.78,  6.82,  6.84,  6.88,
    6.92,  6.94,  6.98,  7.00,  7.04,  7.08,  7.12,  7.14,  7.18,  7.22,
    7.26,  7.28,  7.30,  7.34,  7.38,  7.42,  7.46,  7.50,  7.54,  7.58,
    7.62,  7.66,  7.70,  7.74,  7.78,  7.82,  7.86,  7.90,  7.92,  7.98,
    8.02,  8.06,  8.10,  8.16,  8.20,  8.26,  8.30,  8.34,  8.40,  8.44,
    8.50,  8.54,  8.60,  8.66,  8.68,  8.74,  8.78,  8.84,  8.90,  8.96,
    9.02,  9.08,  9.14,  9.20,  9.26,  9.32,  9.38,  9.44,  9.52,  9.58,
    9.62,  9.68,  9.76,  9.82,  9.90,  9.96,  10.04, 10.12, 10.18, 10.26,
    10.34, 10.42, 10.50, 10.58, 10.68, 10.76, 10.80, 10.88, 10.98, 11.08,
    11.16, 11.26, 11.36, 11.44, 11.54, 11.64, 11.74, 11.86, 11.96, 12.08,
    12.18, 12.30, 12.36, 12.48, 12.60, 12.72, 12.84, 12.96, 13.10, 13.24,
    13.36, 13.50, 13.64, 13.78, 13.94, 14.08, 14.24, 14.40, 14.48, 14.66,
    14.82, 15.00, 15.18, 15.36, 15.54, 15.74, 15.92, 16.12, 16.32, 16.54,
    16.76, 16.98, 17.22, 17.44, 17.58, 17.82, 18.08, 18.34, 18.62, 18.90,
    19.20, 19.48, 19.80, 20.10, 20.44, 20.78, 21.12, 21.50, 21.88, 22.26,
    22.48, 22.90, 23.34, 23.80, 24.26, 24.74, 25.26, 25.76, 26.32, 26.86,
    27.48, 28.12, 28.78, 29.50, 29.50, 29.50
  }
};

template <typename T> property<T>& property_alias(uhd::property_tree::sptr& _tree,
    const uhd::fs_path& orig, const uhd::fs_path& alias)
{
    return _tree->create<T>(alias)
        .subscribe(boost::bind(&uhd::property<T>::set, boost::ref(_tree->access<T>(orig)), boost::placeholders::_1))
        .publish(boost::bind(&uhd::property<T>::get, boost::ref(_tree->access<T>(orig))));
}

static device::sptr umtrx_make(const device_addr_t& device_addr) {
    return device::sptr(new umtrx_impl(device_addr));
}

device_addrs_t umtrx_find(const device_addr_t& hint);

UHD_STATIC_BLOCK(register_umtrx_device) {
#ifdef UHD_HAS_DEVICE_FILTER
    device::register_device(&umtrx_find, &umtrx_make, device::USRP);
#else
    device::register_device(&umtrx_find, &umtrx_make);
#endif
}

struct mtu_result_t {
    size_t recv_mtu, send_mtu;
};

static std::vector<std::string> power_sensors =
boost::assign::list_of("PR1")("PF1")("PR2")("PF2");

static std::vector<std::string> dc_sensors =
boost::assign::list_of("zero")("Vin")("VinPA")("DCOUT");

static mtu_result_t determine_mtu(const std::string& addr, const mtu_result_t& user_mtu) {
    udp_simple::sptr udp_sock = udp_simple::make_connected(
        addr, BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
    );

    std::vector<boost::uint8_t> buffer(std::max(user_mtu.recv_mtu, user_mtu.send_mtu));
    usrp2_ctrl_data_t* ctrl_data = reinterpret_cast<usrp2_ctrl_data_t*>(&buffer.front());
    static const double echo_timeout = 0.020;

    ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
    ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
    ctrl_data->data.echo_args.len = htonl(sizeof(usrp2_ctrl_data_t));
    udp_sock->send(boost::asio::buffer(buffer, sizeof(usrp2_ctrl_data_t)));
    udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);
    if (ntohl(ctrl_data->id) != USRP2_CTRL_ID_HOLLER_BACK_DUDE)
        throw uhd::not_implemented_error("holler protocol not implemented");

    size_t min_recv_mtu = sizeof(usrp2_ctrl_data_t), max_recv_mtu = user_mtu.recv_mtu;
    size_t min_send_mtu = sizeof(usrp2_ctrl_data_t), max_send_mtu = user_mtu.send_mtu;

    while (min_recv_mtu < max_recv_mtu) {

        size_t test_mtu = (max_recv_mtu / 2 + min_recv_mtu / 2 + 3) & ~3;

        ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
        ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
        ctrl_data->data.echo_args.len = htonl(test_mtu);
        udp_sock->send(boost::asio::buffer(buffer, sizeof(usrp2_ctrl_data_t)));

        size_t len = udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);

        if (len >= test_mtu) min_recv_mtu = test_mtu;
        else                 max_recv_mtu = test_mtu - 4;

    }

    while (min_send_mtu < max_send_mtu) {

        size_t test_mtu = (max_send_mtu / 2 + min_send_mtu / 2 + 3) & ~3;

        ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
        ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
        ctrl_data->data.echo_args.len = htonl(sizeof(usrp2_ctrl_data_t));
        udp_sock->send(boost::asio::buffer(buffer, test_mtu));

        size_t len = udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);
        if (len >= sizeof(usrp2_ctrl_data_t)) len = ntohl(ctrl_data->data.echo_args.len);

        if (len >= test_mtu) min_send_mtu = test_mtu;
        else                 max_send_mtu = test_mtu - 4;
    }

    mtu_result_t mtu;
    mtu.recv_mtu = min_recv_mtu;
    mtu.send_mtu = min_send_mtu;
    return mtu;
}

umtrx_impl::umtrx_impl(const device_addr_t& device_addr)
{
    _umtrx_vga2_def = device_addr.cast<int>("lmsvga2", UMTRX_VGA2_DEF);
    _device_ip_addr = device_addr["addr"];
    UHD_LOGGER_INFO("UMTRX") << "UmTRX driver version: " << UMTRX_VERSION;
    UHD_LOGGER_INFO("UMTRX") << "Opening a UmTRX device... " << _device_ip_addr;

    mtu_result_t user_mtu;
    user_mtu.recv_mtu = size_t(device_addr.cast<double>("recv_frame_size", udp_simple::mtu));
    user_mtu.send_mtu = size_t(device_addr.cast<double>("send_frame_size", udp_simple::mtu));
    user_mtu = determine_mtu(_device_ip_addr, user_mtu);
    UHD_LOGGER_INFO("UMTRX") << "recv_mtu: " << user_mtu.recv_mtu;
    UHD_LOGGER_INFO("UMTRX") << "send_mtu: " << user_mtu.send_mtu;

    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("UmTRX Device");
    const fs_path mb_path = "/mboards/0";

    _iface = umtrx_iface::make(udp_simple::make_connected(
        _device_ip_addr, BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
    ));
    _tree->create<std::string>(mb_path / "name").set(_iface->get_cname());
    _tree->create<std::string>(mb_path / "fw_version").set(_iface->get_fw_version_string());

    _tree->create<uhd::wb_iface::sptr>(mb_path / "wb_iface").set(_iface);
    _tree->create<uhd::spi_iface::sptr>(mb_path / "spi_iface").set(_iface);
    _tree->create<uhd::i2c_iface::sptr>(mb_path / "i2c_iface").set(_iface);

    const boost::uint32_t fpga_compat_num = _iface->peek32(U2_REG_COMPAT_NUM_RB);
    const boost::uint16_t fpga_major = fpga_compat_num >> 16, fpga_minor = fpga_compat_num & 0xffff;
    if (fpga_major != USRP2_FPGA_COMPAT_NUM) {
        throw uhd::runtime_error(str(boost::format(
            "\nPlease update the firmware and FPGA images for your device.\n"
            "See the application notes for UmTRX for instructions.\n"
            "Expected FPGA compatibility number %d, but got %d:\n"
            "The FPGA build is not compatible with the host code build."
        ) % int(USRP2_FPGA_COMPAT_NUM) % fpga_major));
    }
    _tree->create<std::string>(mb_path / "fpga_version").set(str(boost::format("%u.%u") % fpga_major % fpga_minor));

    _iface->lock_device(true);

    _iface->poke32(U2_REG_MISC_CTRL_SFC_CLEAR, 1);
    const size_t fifo_ctrl_window(device_addr.cast<size_t>("fifo_ctrl_window", 1024));
    _ctrl = umtrx_fifo_ctrl::make(this->make_xport(UMTRX_CTRL_FRAMER, device_addr_t()), UMTRX_CTRL_SID, fifo_ctrl_window);
    _ctrl->peek32(0);
    _tree->create<time_spec_t>(mb_path / "time/cmd")
        .subscribe(boost::bind(&umtrx_fifo_ctrl::set_time, _ctrl, boost::placeholders::_1));
    _tree->create<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&umtrx_fifo_ctrl::set_tick_rate, _ctrl, boost::placeholders::_1));

    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(_iface->mb_eeprom)
        .subscribe(boost::bind(&umtrx_impl::set_mb_eeprom, this, _iface, boost::placeholders::_1));

    _tree->access<double>(mb_path / "tick_rate")
        .publish(boost::bind(&umtrx_impl::get_master_clock_rate, this))
        .subscribe(boost::bind(&umtrx_impl::update_tick_rate, this, boost::placeholders::_1));
    _tree->create<double>(mb_path / "dsp_rate")
        .publish(boost::bind(&umtrx_impl::get_master_dsp_rate, this));

    _iface->poke32(U2_REG_MISC_LMS_RES, LMS1_RESET | LMS2_RESET);
    _iface->poke32(U2_REG_MISC_LMS_RES, 0);
    _iface->poke32(U2_REG_MISC_LMS_RES, LMS1_RESET | LMS2_RESET);

    detect_hw_rev(mb_path);
    _tree->create<std::string>(mb_path / "hwrev").set(get_hw_rev());
    UHD_LOGGER_INFO("UMTRX") << "Detected UmTRX " << get_hw_rev();

    _hw_dcdc_ver = device_addr.cast<int>("dcdc_ver", -1);
    if (_hw_dcdc_ver < 0)
    {
        detect_hw_dcdc_ver(mb_path);
    }
    else {
        UHD_ASSERT_THROW(_hw_dcdc_ver < DCDC_VER_COUNT);
        UHD_LOGGER_INFO("UMTRX") << "Using DCDC version " << _hw_dcdc_ver;
    }
    _tree->create<int>(mb_path / "hwdcdc_ver").set(_hw_dcdc_ver);

    const std::string detect_umsel = device_addr.get("umsel", "off");
    if (detect_umsel != "off")
    {
        const bool umsel_verbose = device_addr.has_key("umsel_verbose");
        _umsel2 = umsel2_ctrl::make(_ctrl/*peek*/, _ctrl/*spi*/, this->get_master_clock_rate(), umsel_verbose);
    }

    if (_umsel2)
    {
        _tree->create<sensor_value_t>(mb_path / "dboards" / "A" / "rx_frontends" / "0" / "sensors" / "aux_lo_locked")
            .publish(boost::bind(&umsel2_ctrl::get_locked, _umsel2, 1));
        _tree->create<sensor_value_t>(mb_path / "dboards" / "B" / "rx_frontends" / "0" / "sensors" / "aux_lo_locked")
            .publish(boost::bind(&umsel2_ctrl::get_locked, _umsel2, 2));
    }

    _tree->create<bool>(mb_path / "divsw1")
        .subscribe(boost::bind(&umtrx_impl::set_diversity, this, boost::placeholders::_1, 0))
        .set(device_addr.cast<bool>("divsw1", false));
    UHD_LOGGER_INFO("UMTRX") << "Diversity switch for channel 1: "
        << (_tree->access<bool>(mb_path / "divsw1").get() ? "true" : "false");
    _tree->create<bool>(mb_path / "divsw2")
        .subscribe(boost::bind(&umtrx_impl::set_diversity, this, boost::placeholders::_1, 1))
        .set(device_addr.cast<bool>("divsw2", false));
    UHD_LOGGER_INFO("UMTRX") << "Diversity switch for channel 2: "
        << (_tree->access<bool>(mb_path / "divsw2").get() ? "true" : "false");

    _pll_div = 1;

    std::list<std::string> pa_types = power_amp::list_pa_str();
    std::string pa_list_str;
    BOOST_FOREACH(const std::string & pa_str, pa_types)
    {
        pa_list_str += pa_str + " ";
    }
    UHD_LOGGER_INFO("UMTRX") << "Known PA types: " << pa_list_str;

    power_amp::pa_type_t pa_type = power_amp::pa_str_to_type(device_addr.cast<std::string>("pa", "NONE"));
    if (_hw_rev < UMTRX_VER_2_3_1 and pa_type != power_amp::PA_NONE)
    {
        UHD_LOGGER_ERROR("UMTRX") << "PA type " << power_amp::pa_type_to_str(pa_type) << " is not supported for UmTRX "
            << get_hw_rev() << ". Setting PA type to NONE.";
        pa_type = power_amp::PA_NONE;
    }

    for (char name = 'A'; name <= 'B'; name++)
    {
        std::string name_str = std::string(1, name);
        _pa[name_str] = power_amp::make(pa_type);
        UHD_LOGGER_INFO("UMTRX") << "Installed PA for side" << name_str << ": " << power_amp::pa_type_to_str(pa_type);
    }

    if (_pa["A"])
    {
        _pa_power_max_dBm = _pa["A"]->max_power_dBm();

        double limit_w = device_addr.cast<double>("pa_power_max_w", _pa["A"]->max_power_w());
        if (limit_w != _pa["A"]->max_power_w()) {
            _pa_power_max_dBm = power_amp::w2dBm(limit_w);
        }

        double limit_dbm = device_addr.cast<double>("pa_power_max_dbm", _pa["A"]->max_power_dBm());
        if (limit_dbm != _pa["A"]->max_power_dBm()) {
            _pa_power_max_dBm = limit_dbm;
        }

        if (_pa_power_max_dBm != _pa["A"]->max_power_dBm()) {
            UHD_LOGGER_INFO("UMTRX") << "Limiting PA output power to: " << _pa_power_max_dBm << "dBm (" << power_amp::dBm2w(_pa_power_max_dBm) << "W)";
        }
    }

    for (char name = 'A'; name <= 'B'; name++)
    {
        const fs_path rx_codec_path = mb_path / ("rx_codecs") / std::string(1, name);
        _tree->create<std::string>(rx_codec_path / "name").set("RX LMS ADC");
        _tree->create<int>(rx_codec_path / "gains");

        const fs_path tx_codec_path = mb_path / ("tx_codecs") / std::string(1, name);
        _tree->create<std::string>(tx_codec_path / "name").set("TX LMS DAC");
        _tree->create<int>(tx_codec_path / "gains");
    }

    _rx_fes.resize(2);
    _tx_fes.resize(2);
    _rx_fes[0] = rx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_FRONT0));
    _rx_fes[1] = rx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_FRONT1));
    _tx_fes[0] = tx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_FRONT0));
    _tx_fes[1] = tx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_FRONT1));

    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .subscribe(boost::bind(&umtrx_impl::update_rx_subdev_spec, this, boost::placeholders::_1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .subscribe(boost::bind(&umtrx_impl::update_tx_subdev_spec, this, boost::placeholders::_1));

    for (char name = 'A'; name <= 'B'; name++)
    {
        const std::string fe_name = std::string(1, name);
        const fs_path rx_fe_path = mb_path / "rx_frontends" / fe_name;
        const fs_path tx_fe_path = mb_path / "tx_frontends" / fe_name;
        const rx_frontend_core_200::sptr rx_fe = (fe_name == "A") ? _rx_fes[0] : _rx_fes[1];
        const tx_frontend_core_200::sptr tx_fe = (fe_name == "A") ? _tx_fes[0] : _tx_fes[1];

        tx_fe->set_mux("IQ");
        rx_fe->set_mux(false/*no swap*/);
        _tree->create<std::complex<double> >(rx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&rx_frontend_core_200::set_dc_offset, rx_fe, boost::placeholders::_1))
            .set(std::complex<double>(0.0, 0.0));
        _tree->create<bool>(rx_fe_path / "dc_offset" / "enable")
            .subscribe(boost::bind(&rx_frontend_core_200::set_dc_offset_auto, rx_fe, boost::placeholders::_1))
            .set(true);
        _tree->create<std::complex<double> >(rx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&rx_frontend_core_200::set_iq_balance, rx_fe, boost::placeholders::_1))
            .set(std::polar<double>(0.0, 0.0));
        _tree->create<std::complex<double> >(tx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&tx_frontend_core_200::set_iq_balance, tx_fe, boost::placeholders::_1))
            .set(std::polar<double>(0.0, 0.0));
    }

    _rx_dsps.resize(_iface->peek32(U2_REG_NUM_DDC));
    if (_rx_dsps.size() < 2) throw uhd::runtime_error(str(boost::format("umtrx rx_dsps %u -- (unsupported FPGA image?)") % _rx_dsps.size()));
    if (_rx_dsps.size() > 0) _rx_dsps[0] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP0), U2_REG_SR_ADDR(SR_RX_CTRL0), UMTRX_DSP_RX0_SID, true);
    if (_rx_dsps.size() > 1) _rx_dsps[1] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP1), U2_REG_SR_ADDR(SR_RX_CTRL1), UMTRX_DSP_RX1_SID, true);
    if (_rx_dsps.size() > 2) _rx_dsps[2] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP2), U2_REG_SR_ADDR(SR_RX_CTRL2), UMTRX_DSP_RX2_SID, true);
    if (_rx_dsps.size() > 3) _rx_dsps[3] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP3), U2_REG_SR_ADDR(SR_RX_CTRL3), UMTRX_DSP_RX3_SID, true);
    _tree->create<sensor_value_t>(mb_path / "rx_dsps");

    for (size_t dspno = 0; dspno < _rx_dsps.size(); dspno++) {
        _rx_dsps[dspno]->set_mux("IQ", false/*no swap*/);
        _rx_dsps[dspno]->set_link_rate(UMTRX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "dsp_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_tick_rate, _rx_dsps[dspno], boost::placeholders::_1));
        _tree->access<double>(mb_path / "tick_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_vita_rate, _rx_dsps[dspno], boost::placeholders::_1));
        fs_path rx_dsp_path = mb_path / str(boost::format("rx_dsps/%u") % dspno);
        _tree->create<meta_range_t>(rx_dsp_path / "rate/range")
            .publish(boost::bind(&rx_dsp_core_200::get_host_rates, _rx_dsps[dspno]));
        _tree->create<double>(rx_dsp_path / "rate/value")
            .set(this->get_master_clock_rate() / 12)
            .coerce(boost::bind(&rx_dsp_core_200::set_host_rate, _rx_dsps[dspno], boost::placeholders::_1))
            .subscribe(boost::bind(&umtrx_impl::update_rx_samp_rate, this, dspno, boost::placeholders::_1));
        _tree->create<double>(rx_dsp_path / "freq/value")
            .coerce(boost::bind(&rx_dsp_core_200::set_freq, _rx_dsps[dspno], boost::placeholders::_1));
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range")
            .publish(boost::bind(&rx_dsp_core_200::get_freq_range, _rx_dsps[dspno]));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
            .subscribe(boost::bind(&rx_dsp_core_200::issue_stream_command, _rx_dsps[dspno], boost::placeholders::_1));
    }

    _tx_dsps.resize(_iface->peek32(U2_REG_NUM_DUC));
    if (_tx_dsps.empty()) _tx_dsps.resize(1);
    if (_tx_dsps.size() > 0) _tx_dsps[0] = tx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_DSP0), U2_REG_SR_ADDR(SR_TX_CTRL0), UMTRX_DSP_TX0_SID);
    if (_tx_dsps.size() > 1) _tx_dsps[1] = tx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_DSP1), U2_REG_SR_ADDR(SR_TX_CTRL1), UMTRX_DSP_TX1_SID);
    _tree->create<sensor_value_t>(mb_path / "tx_dsps");

    for (size_t dspno = 0; dspno < _tx_dsps.size(); dspno++) {
        _tx_dsps[dspno]->set_link_rate(UMTRX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "dsp_rate")
            .subscribe(boost::bind(&tx_dsp_core_200::set_tick_rate, _tx_dsps[dspno], boost::placeholders::_1));
        fs_path tx_dsp_path = mb_path / str(boost::format("tx_dsps/%u") % dspno);
        _tree->create<meta_range_t>(tx_dsp_path / "rate/range")
            .publish(boost::bind(&tx_dsp_core_200::get_host_rates, _tx_dsps[dspno]));
        _tree->create<double>(tx_dsp_path / "rate/value")
            .set(this->get_master_clock_rate() / 12)
            .coerce(boost::bind(&tx_dsp_core_200::set_host_rate, _tx_dsps[dspno], boost::placeholders::_1))
            .subscribe(boost::bind(&umtrx_impl::update_tx_samp_rate, this, dspno, boost::placeholders::_1));
        _tree->create<double>(tx_dsp_path / "freq/value")
            .coerce(boost::bind(&tx_dsp_core_200::set_freq, _tx_dsps[dspno], boost::placeholders::_1));
        _tree->create<meta_range_t>(tx_dsp_path / "freq/range")
            .publish(boost::bind(&tx_dsp_core_200::get_freq_range, _tx_dsps[dspno]));
    }

    time64_core_200::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_hi_now = U2_REG_TIME64_HI_RB_IMM;
    time64_rb_bases.rb_lo_now = U2_REG_TIME64_LO_RB_IMM;
    time64_rb_bases.rb_hi_pps = U2_REG_TIME64_HI_RB_PPS;
    time64_rb_bases.rb_lo_pps = U2_REG_TIME64_LO_RB_PPS;
    _time64 = time64_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TIME64), time64_rb_bases);

    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&time64_core_200::set_tick_rate, _time64, boost::placeholders::_1));
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time64_core_200::get_time_now, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_now, _time64, boost::placeholders::_1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time64_core_200::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_next_pps, _time64, boost::placeholders::_1));
    _tree->create<std::string>(mb_path / "time_source" / "value")
        .subscribe(boost::bind(&time64_core_200::set_time_source, _time64, boost::placeholders::_1));
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options")
        .publish(boost::bind(&time64_core_200::get_time_sources, _time64));
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&umtrx_impl::update_clock_source, this, boost::placeholders::_1));

    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_sources);

    _lms_ctrl["A"] = lms6002d_ctrl::make(_ctrl/*spi*/, SPI_SS_LMS1, this->get_master_clock_rate() / _pll_div);
    _lms_ctrl["B"] = lms6002d_ctrl::make(_ctrl/*spi*/, SPI_SS_LMS2, this->get_master_clock_rate() / _pll_div);

    dboard_eeprom_t rx_db_eeprom, tx_db_eeprom, gdb_db_eeprom;
    rx_db_eeprom.id = 0xfa07;
    rx_db_eeprom.revision = _iface->mb_eeprom["revision"];
    tx_db_eeprom.id = 0xfa09;
    tx_db_eeprom.revision = _iface->mb_eeprom["revision"];

    BOOST_FOREACH(const std::string & fe_name, _lms_ctrl.keys())
    {
        lms6002d_ctrl::sptr ctrl = _lms_ctrl[fe_name];

        const fs_path rx_rf_fe_path = mb_path / "dboards" / fe_name / "rx_frontends" / "0";
        const fs_path tx_rf_fe_path = mb_path / "dboards" / fe_name / "tx_frontends" / "0";

        _tree->create<std::string>(rx_rf_fe_path / "name").set("LMS6002D");
        _tree->create<std::string>(tx_rf_fe_path / "name").set("LMS6002D");

        rx_db_eeprom.serial = _iface->mb_eeprom["serial"] + "." + fe_name;
        tx_db_eeprom.serial = _iface->mb_eeprom["serial"] + "." + fe_name;
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / fe_name / "rx_eeprom")
            .set(rx_db_eeprom);
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / fe_name / "tx_eeprom")
            .set(tx_db_eeprom);
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / fe_name / "gdb_eeprom")
            .set(gdb_db_eeprom);

        _tree->create<sensor_value_t>(rx_rf_fe_path / "sensors" / "lo_locked")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_pll_locked, ctrl));
        _tree->create<sensor_value_t>(tx_rf_fe_path / "sensors" / "lo_locked")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_pll_locked, ctrl));

        BOOST_FOREACH(const std::string & name, ctrl->get_rx_gains())
        {
            _tree->create<meta_range_t>(rx_rf_fe_path / "gains" / name / "range")
                .publish(boost::bind(&lms6002d_ctrl::get_rx_gain_range, ctrl, name));

            _tree->create<double>(rx_rf_fe_path / "gains" / name / "value")
                .coerce(boost::bind(&lms6002d_ctrl::set_rx_gain, ctrl, boost::placeholders::_1, name))
                .set((ctrl->get_rx_gain_range(name).start() + ctrl->get_rx_gain_range(name).stop()) / 2.0);
        }

        if (!_pa[fe_name])
        {
            BOOST_FOREACH(const std::string & name, ctrl->get_tx_gains())
            {
                _tree->create<meta_range_t>(tx_rf_fe_path / "gains" / name / "range")
                    .publish(boost::bind(&lms6002d_ctrl::get_tx_gain_range, ctrl, name));

                _tree->create<double>(tx_rf_fe_path / "gains" / name / "value")
                    .coerce(boost::bind(&lms6002d_ctrl::set_tx_gain, ctrl, boost::placeholders::_1, name))
                    .set((ctrl->get_tx_gain_range(name).start() + ctrl->get_tx_gain_range(name).stop()) / 2.0);
            }
        }
        else {
            const int vga1 = device_addr.cast<int>("lmsvga1", UMTRX_VGA1_DEF);
            ctrl->set_tx_gain(vga1, "VGA1");
            _tx_power_range[fe_name] = generate_tx_power_range(fe_name);

            _tree->create<meta_range_t>(tx_rf_fe_path / "gains" / "PA" / "range")
                .publish(boost::bind(&umtrx_impl::get_tx_power_range, this, fe_name));

            _tree->create<double>(tx_rf_fe_path / "gains" / "PA" / "value")
                .coerce(boost::bind(&umtrx_impl::set_tx_power, this, boost::placeholders::_1, fe_name))
                .set(get_tx_power_range(fe_name).stop());

        }

        _tree->create<double>(rx_rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&umtrx_impl::set_rx_freq, this, fe_name, boost::placeholders::_1));
        _tree->create<meta_range_t>(rx_rf_fe_path / "freq" / "range")
            .publish(boost::bind(&umtrx_impl::get_rx_freq_range, this, fe_name));
        _tree->create<bool>(rx_rf_fe_path / "use_lo_offset").set(false);

        _tree->create<double>(tx_rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_tx_freq, ctrl, boost::placeholders::_1));
        _tree->create<meta_range_t>(tx_rf_fe_path / "freq" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_freq_range, ctrl));
        _tree->create<bool>(tx_rf_fe_path / "use_lo_offset").set(false);

        _tree->create<std::vector<std::string> >(rx_rf_fe_path / "antenna" / "options")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_antennas, ctrl));
        _tree->create<std::string>(rx_rf_fe_path / "antenna" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::set_rx_ant, ctrl, boost::placeholders::_1))
            .set("RX1");

        _tree->create<std::vector<std::string> >(tx_rf_fe_path / "antenna" / "options")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_antennas, ctrl));
        _tree->create<std::string>(tx_rf_fe_path / "antenna" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::set_tx_ant, ctrl, boost::placeholders::_1))
            .set("TX2");

        _tree->create<std::string>(rx_rf_fe_path / "connection").set("IQ");
        _tree->create<std::string>(tx_rf_fe_path / "connection").set("IQ");
        _tree->create<bool>(rx_rf_fe_path / "enabled")
            .coerce(boost::bind(&lms6002d_ctrl::set_rx_enabled, ctrl, boost::placeholders::_1));
        _tree->create<bool>(tx_rf_fe_path / "enabled")
            .coerce(boost::bind(&lms6002d_ctrl::set_tx_enabled, ctrl, boost::placeholders::_1));

        _tree->create<double>(rx_rf_fe_path / "bandwidth" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_rx_bandwidth, ctrl, boost::placeholders::_1))
            .set(2 * 0.75e6);
        _tree->create<meta_range_t>(rx_rf_fe_path / "bandwidth" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_bw_range, ctrl));

        _tree->create<double>(tx_rf_fe_path / "bandwidth" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_tx_bandwidth, ctrl, boost::placeholders::_1))
            .set(2 * 0.75e6);
        _tree->create<meta_range_t>(tx_rf_fe_path / "bandwidth" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_bw_range, ctrl));

        _tree->access<double>(tx_rf_fe_path / "freq" / "value")
            .set(0.0)
            .subscribe(boost::bind(&umtrx_impl::set_tx_fe_corrections, this, "0", fe_name, boost::placeholders::_1));
        _tree->access<double>(rx_rf_fe_path / "freq" / "value")
            .set(0.0)
            .subscribe(boost::bind(&umtrx_impl::set_rx_fe_corrections, this, "0", fe_name, boost::placeholders::_1));

        _tree->create<uint8_t>(tx_rf_fe_path / "lms6002d" / "tx_dc_i" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::_set_tx_vga1dc_i_int, ctrl, boost::placeholders::_1))
            .publish(boost::bind(&lms6002d_ctrl::get_tx_vga1dc_i_int, ctrl));
        _tree->create<uint8_t>(tx_rf_fe_path / "lms6002d" / "tx_dc_q" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::_set_tx_vga1dc_q_int, ctrl, boost::placeholders::_1))
            .publish(boost::bind(&lms6002d_ctrl::get_tx_vga1dc_q_int, ctrl));

        std::string tx_name = (fe_name == "A") ? "tx1" : "tx2";
        const std::string dc_i_str = _iface->mb_eeprom.get(tx_name + "-vga1-dc-i", "");
        const std::string dc_q_str = _iface->mb_eeprom.get(tx_name + "-vga1-dc-q", "");
        double dc_i = dc_i_str.empty() ? 0.0 : dc_offset_int2double(boost::lexical_cast<int>(dc_i_str));
        double dc_q = dc_q_str.empty() ? 0.0 : dc_offset_int2double(boost::lexical_cast<int>(dc_q_str));

        _tree->create<std::complex<double> >(mb_path / "tx_frontends" / fe_name / "dc_offset" / "value")
            .publish(boost::bind(&umtrx_impl::get_dc_offset_correction, this, fe_name))
            .subscribe(boost::bind(&umtrx_impl::set_dc_offset_correction, this, fe_name, boost::placeholders::_1))
            .set(std::complex<double>(dc_i, dc_q));

        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_fe_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxfe_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxfe_dc_i, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_fe_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxfe_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxfe_dc_q, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_lpf_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxlpf_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxlpf_dc_i, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_lpf_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxlpf_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxlpf_dc_q, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2_dc_reference" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2_dc_reference, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2_dc_reference, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2a_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2a_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2a_dc_i, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2a_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2a_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2a_dc_q, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2b_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2b_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2b_dc_i, ctrl, boost::placeholders::_1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2b_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2b_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2b_dc_q, ctrl, boost::placeholders::_1));

        property_alias<bool>(_tree, mb_path / "divsw" + (fe_name == "A" ? "1" : "2"), rx_rf_fe_path / "diversity");
    }

    _tree->create<uint16_t>(mb_path / "tcxo_dac" / "value")
        .subscribe(boost::bind(&umtrx_impl::set_tcxo_dac, this, _iface, boost::placeholders::_1));

    _tree->access<double>(mb_path / "tick_rate")
        .set(this->get_master_clock_rate());
    _tree->access<double>(mb_path / "dsp_rate")
        .set(this->get_master_dsp_rate());
    this->time64_self_test();

    BOOST_FOREACH(const std::string & name, _tree->list(mb_path / "rx_dsps"))
    {
        _tree->access<double>(mb_path / "rx_dsps" / name / "freq" / "value").set(0.0);
    }
    BOOST_FOREACH(const std::string & name, _tree->list(mb_path / "tx_dsps"))
    {
        _tree->access<double>(mb_path / "tx_dsps" / name / "freq" / "value").set(0.0);
    }

    _rx_streamers.resize(_rx_dsps.size());
    _tx_streamers.resize(_tx_dsps.size());

    subdev_spec_t rx_spec("A:0 B:0 A:0 B:0");
    rx_spec.resize(_rx_dsps.size());
    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(rx_spec);

    subdev_spec_t tx_spec("A:0 B:0 A:0 B:0");
    tx_spec.resize(_tx_dsps.size());
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(tx_spec);

    _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
    _tree->access<std::string>(mb_path / "time_source" / "value").set("none");

    this->status_monitor_start(device_addr);
}

umtrx_impl::~umtrx_impl(void)
{
    this->status_monitor_stop();

    BOOST_FOREACH(const std::string & fe_name, _lms_ctrl.keys())
    {
        lms6002d_ctrl::sptr ctrl = _lms_ctrl[fe_name];
        try
        {
            ctrl->set_rx_enabled(false);
            ctrl->set_tx_enabled(false);
        }
        catch (...) {}
    }
}

int umtrx_impl::volt_to_dcdc_r(double v)
{
    if (v <= _dcdc_val_to_volt[_hw_dcdc_ver][0])
        return 0;
    else if (v >= _dcdc_val_to_volt[_hw_dcdc_ver][255])
        return 255;
    else
        return std::lower_bound(&_dcdc_val_to_volt[_hw_dcdc_ver][0], &_dcdc_val_to_volt[_hw_dcdc_ver][256], v) -
        &_dcdc_val_to_volt[_hw_dcdc_ver][0];
}

void umtrx_impl::set_pa_dcdc_r(uint8_t val)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    if (_hw_rev >= UMTRX_VER_2_3_1)
    {
        _pa_dcdc_r = val;
        _iface->write_i2c(BOOST_BINARY(0101100), boost::assign::list_of(0)(val));
    }
}

uhd::gain_range_t umtrx_impl::generate_pa_power_range(const std::string& which) const
{
    double min_power = _pa[which]->min_power_dBm();
    double max_power = _pa_power_max_dBm;
    return uhd::gain_range_t(min_power, max_power, 0.1);
}

const uhd::gain_range_t& umtrx_impl::get_tx_power_range(const std::string& which) const
{
    return _tx_power_range[which];
}

double umtrx_impl::set_tx_power(double power, const std::string& which)
{
    double min_pa_power = _pa[which]->min_power_dBm();
    double actual_power;

    if (power >= min_pa_power)
    {
        UHD_LOGGER_INFO("UMTRX") << "Setting Tx power using PA (VGA2=" << _umtrx_vga2_def << ", PA=" << power << ")";
        _lms_ctrl[which]->set_tx_gain(_umtrx_vga2_def, "VGA2");
        actual_power = set_pa_power(power, which);
    }
    else {
        double vga2_gain = _umtrx_vga2_def - (min_pa_power - power);
        UHD_LOGGER_INFO("UMTRX") << "Setting Tx power using VGA2 (VGA2=" << vga2_gain << ", PA=" << min_pa_power << ")";
        actual_power = _lms_ctrl[which]->set_tx_gain(vga2_gain, "VGA2");
        actual_power = set_pa_power(min_pa_power, which) - (_umtrx_vga2_def - actual_power);
    }

    return actual_power;
}

double umtrx_impl::set_pa_power(double power, const std::string& which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);

    double v = _pa[which]->dBm2v(power);
    uint8_t dcdc_val = volt_to_dcdc_r(v);
    set_nlow(true);
    set_pa_dcdc_r(dcdc_val);

    double v_actual = read_dc_v("DCOUT").to_real();
    double power_actual = _pa[which]->v2dBm(v_actual);

    UHD_LOGGER_INFO("UMTRX") << "Setting PA power: Requested: " << power << "dBm = " << power_amp::dBm2w(power) << "W "
        << "(" << v << "V dcdc_r=" << int(dcdc_val) << "). "
        << "Actual: " << power_actual << "dBm = " << power_amp::dBm2w(power_actual) << "W "
        << "(" << v_actual << "V)";

    return power_actual;
}

void umtrx_impl::set_mb_eeprom(const uhd::i2c_iface::sptr& iface, const uhd::usrp::mboard_eeprom_t& eeprom)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    store_umtrx_eeprom(eeprom, *iface);
}

void umtrx_impl::time64_self_test(void)
{
    UHD_LOGGER_INFO("UMTRX") << "Time register self-test... ";
    const time_spec_t t0 = _time64->get_time_now();
    const double sleepTime = 0.50;
    boost::this_thread::sleep(boost::posix_time::milliseconds(long(sleepTime * 1000)));
    const time_spec_t t1 = _time64->get_time_now();
    const double secs_elapsed = (t1 - t0).get_real_secs();
    const bool within_range = (secs_elapsed < (1.5) * sleepTime and secs_elapsed >(0.5) * sleepTime);
    UHD_LOGGER_INFO("UMTRX") << (within_range ? "pass" : "fail");
}

void umtrx_impl::update_clock_source(const std::string&) {}

void umtrx_impl::set_rx_fe_corrections(const std::string& mb, const std::string& board, const double lo_freq) {
    apply_rx_fe_corrections(this->get_tree()->subtree("/mboards/" + mb), board, lo_freq);
}

void umtrx_impl::set_tx_fe_corrections(const std::string& mb, const std::string& board, const double lo_freq) {
    apply_tx_fe_corrections(this->get_tree()->subtree("/mboards/" + mb), board, lo_freq);
}

void umtrx_impl::set_tcxo_dac(const umtrx_iface::sptr& iface, const uint16_t val) {
    if (verbosity > 0) printf("umtrx_impl::set_tcxo_dac(%d)\n", val);
    iface->send_zpu_action(UMTRX_ZPU_REQUEST_SET_VCTCXO_DAC, val);
}

uint16_t umtrx_impl::get_tcxo_dac(const umtrx_iface::sptr& iface) {
    uint16_t val = iface->send_zpu_action(UMTRX_ZPU_REQUEST_GET_VCTCXO_DAC, 0);
    if (verbosity > 0) printf("umtrx_impl::get_tcxo_dac(): %d\n", val);
    return (uint16_t)val;
}

std::complex<double> umtrx_impl::get_dc_offset_correction(const std::string& which) const
{
    return std::complex<double>(
        dc_offset_int2double(_lms_ctrl[which]->get_tx_vga1dc_i_int()),
        dc_offset_int2double(_lms_ctrl[which]->get_tx_vga1dc_q_int()));
}

void umtrx_impl::set_dc_offset_correction(const std::string& which, const std::complex<double>& corr)
{
    _lms_ctrl[which]->_set_tx_vga1dc_i_int(dc_offset_double2int(corr.real()));
    _lms_ctrl[which]->_set_tx_vga1dc_q_int(dc_offset_double2int(corr.imag()));
}

double umtrx_impl::dc_offset_int2double(uint8_t corr)
{
    return (corr - 128) / 128.0;
}

uint8_t umtrx_impl::dc_offset_double2int(double corr)
{
    return (int)(corr * 128 + 128.5);
}

double umtrx_impl::set_rx_freq(const std::string& which, const double freq)
{
    if (_umsel2)
    {
        const double target_lms_freq = (which == "A") ? UMSEL2_CH1_LMS_IF : UMSEL2_CH2_LMS_IF;
        const double actual_lms_freq = _lms_ctrl[which]->set_rx_freq(target_lms_freq);

        const double target_umsel_freq = freq - actual_lms_freq;
        const double actual_umsel_freq = _umsel2->set_rx_freq((which == "A") ? 1 : 2, target_umsel_freq);

        return actual_umsel_freq + actual_lms_freq;
    }
    else
    {
        return _lms_ctrl[which]->set_rx_freq(freq);
    }
}

uhd::freq_range_t umtrx_impl::get_rx_freq_range(const std::string& which) const
{
    if (_umsel2)
    {
        const double target_lms_freq = (which == "A") ? UMSEL2_CH1_LMS_IF : UMSEL2_CH2_LMS_IF;
        const uhd::freq_range_t range_umsel = _umsel2->get_rx_freq_range((which == "A") ? 1 : 2);
        return uhd::freq_range_t(
            range_umsel.start() + target_lms_freq,
            range_umsel.stop() + target_lms_freq);
    }
    else
    {
        return _lms_ctrl[which]->get_rx_freq_range();
    }
}

uhd::sensor_value_t umtrx_impl::read_temp_c(const std::string& which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    double temp = (which == "A") ? _temp_side_a.get_temp() :
        _temp_side_b.get_temp();
    return uhd::sensor_value_t("Temp" + which, temp, "C");
}

uhd::sensor_value_t umtrx_impl::read_pa_v(const std::string& which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    unsigned i;
    for (i = 0; i < 4; i++) {
        if (which == power_sensors[i])
            break;
    }
    UHD_ASSERT_THROW(i < 4);

    _sense_pwr.set_input((ads1015_ctrl::ads1015_input)
        (ads1015_ctrl::ADS1015_CONF_AIN0_GND + i));
    double val = _sense_pwr.get_value() * 10;
    return uhd::sensor_value_t("Voltage" + which, val, "V");
}

uhd::sensor_value_t umtrx_impl::read_dc_v(const std::string& which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    unsigned i;
    for (i = 0; i < 4; i++) {
        if (which == dc_sensors[i])
            break;
    }
    UHD_ASSERT_THROW(i < 4);

    _sense_dc.set_input((ads1015_ctrl::ads1015_input)
        (ads1015_ctrl::ADS1015_CONF_AIN0_GND + i));
    double val = _sense_dc.get_value() * 40;
    return uhd::sensor_value_t("Voltage" + which, val, "V");
}

void umtrx_impl::detect_hw_rev(const fs_path& mb_path)
{
    if (!tmp102_ctrl::check(_iface, tmp102_ctrl::TMP102_SDA)) {
        _tree->create<sensor_value_t>(mb_path / "sensors");
        _hw_rev = UMTRX_VER_2_0;
        return;
    }
    _temp_side_a.init(_iface, tmp102_ctrl::TMP102_SDA);
    _temp_side_a.set_ex_mode(true);
    _tree->create<sensor_value_t>(mb_path / "sensors" / "tempA")
        .publish(boost::bind(&umtrx_impl::read_temp_c, this, "A"));
    UHD_LOGGER_INFO("UMTRX") << this->read_temp_c("A").to_pp_string();

    if (!tmp102_ctrl::check(_iface, tmp102_ctrl::TMP102_SCL)) {
        _hw_rev = UMTRX_VER_2_1;
        return;
    }
    _temp_side_b.init(_iface, tmp102_ctrl::TMP102_SCL);
    _temp_side_b.set_ex_mode(true);
    _tree->create<sensor_value_t>(mb_path / "sensors" / "tempB")
        .publish(boost::bind(&umtrx_impl::read_temp_c, this, "B"));
    UHD_LOGGER_INFO("UMTRX") << this->read_temp_c("B").to_pp_string();

    if (!ads1015_ctrl::check(_iface, ads1015_ctrl::ADS1015_ADDR_VDD)) {
        _hw_rev = UMTRX_VER_2_2;
        return;
    }
    _sense_pwr.init(_iface, ads1015_ctrl::ADS1015_ADDR_VDD);
    _sense_pwr.set_mode(true);
    _sense_pwr.set_pga(ads1015_ctrl::ADS1015_PGA_2_048V);
    for (unsigned i = 0; i < power_sensors.size(); i++) {
        _tree->create<sensor_value_t>(mb_path / "sensors" / "voltage" + power_sensors[i])
            .publish(boost::bind(&umtrx_impl::read_pa_v, this, power_sensors[i]));
        UHD_LOGGER_INFO("UMTRX") << this->read_pa_v(power_sensors[i]).to_pp_string();
    }

    if (!ads1015_ctrl::check(_iface, ads1015_ctrl::ADS1015_ADDR_GROUND)) {
        _hw_rev = UMTRX_VER_2_3_0;
        return;
    }
    _sense_dc.init(_iface, ads1015_ctrl::ADS1015_ADDR_GROUND);
    _sense_dc.set_mode(true);
    _sense_dc.set_pga(ads1015_ctrl::ADS1015_PGA_1_024V);
    for (unsigned i = 0; i < power_sensors.size(); i++) {
        _tree->create<sensor_value_t>(mb_path / "sensors" / "voltage" + dc_sensors[i])
            .publish(boost::bind(&umtrx_impl::read_dc_v, this, dc_sensors[i]));
        UHD_LOGGER_INFO("UMTRX") << this->read_dc_v(dc_sensors[i]).to_pp_string();
    }

    _hw_rev = UMTRX_VER_2_3_1;
    _tree->create<uint8_t>(mb_path / "pa_dcdc_r")
        .subscribe(boost::bind(&umtrx_impl::set_pa_dcdc_r, this, boost::placeholders::_1));

    std::string pa_dcdc_r = _iface->mb_eeprom.get("pa_dcdc_r", "");
    char* pa_dcdc_r_env = getenv("UMTRX_PA_DCDC_R");
    if (pa_dcdc_r_env) {
        UHD_LOGGER_INFO("UMTRX") << "EEPROM value of pa_dcdc_r:" << pa_dcdc_r.c_str()
            << " is overriden with env UMTRX_PA_DCDC_R:"
            << pa_dcdc_r_env;
        pa_dcdc_r = pa_dcdc_r_env;
    }
    if (pa_dcdc_r.empty())
        set_pa_dcdc_r(0);
    else
        set_pa_dcdc_r(boost::lexical_cast<unsigned>(pa_dcdc_r));

    _pa_en1 = (boost::lexical_cast<int>(_iface->mb_eeprom.get("pa_en1", "1")) == 1);
    _pa_en2 = (boost::lexical_cast<int>(_iface->mb_eeprom.get("pa_en2", "1")) == 1);

    if (getenv("UMTRX_PA_EN1")) _pa_en1 = (boost::lexical_cast<int>(getenv("UMTRX_PA_EN1")) != 0);
    if (getenv("UMTRX_PA_EN2")) _pa_en2 = (boost::lexical_cast<int>(getenv("UMTRX_PA_EN2")) != 0);

    std::string pa_low = _iface->mb_eeprom.get("pa_low", "");
    char* pa_low_env = getenv("UMTRX_PA_LOW");
    if (pa_low_env) {
        UHD_LOGGER_INFO("UMTRX") << "EEPROM value of pa_low:" << pa_low.c_str()
            << " is overriden with env UMTRX_PA_LOW:"
            << pa_low_env;
        pa_low = pa_low_env;
    }
    if (pa_low.empty())
        _pa_nlow = true;
    else
        _pa_nlow = (boost::lexical_cast<int>(pa_low) == 0);

    _tree->create<bool>(mb_path / "pa_en1")
        .subscribe(boost::bind(&umtrx_impl::set_enpa1, this, boost::placeholders::_1));
    _tree->create<bool>(mb_path / "pa_en2")
        .subscribe(boost::bind(&umtrx_impl::set_enpa2, this, boost::placeholders::_1));
    _tree->create<bool>(mb_path / "pa_nlow")
        .subscribe(boost::bind(&umtrx_impl::set_nlow, this, boost::placeholders::_1));

    commit_pa_state();
    UHD_LOGGER_INFO("UMTRX") << "PA low=`" << pa_low.c_str()
        << "` PA dcdc_r=`" << pa_dcdc_r.c_str()
        << "`";
}

void umtrx_impl::commit_pa_state()
{
    if (_hw_rev >= UMTRX_VER_2_3_1)
        _iface->poke32(U2_REG_MISC_LMS_RES, LMS1_RESET | LMS2_RESET
            | PAREG_ENDCSYNC
            | ((_pa_nlow) ? PAREG_NLOW_PA : 0)
            | ((_pa_en1) ? PAREG_ENPA1 : 0)
            | ((_pa_en2) ? PAREG_ENPA2 : 0));
}

void umtrx_impl::set_enpa1(bool en)
{
    _pa_en1 = en; commit_pa_state();
}

void umtrx_impl::set_enpa2(bool en)
{
    _pa_en2 = en; commit_pa_state();
}

void umtrx_impl::set_nlow(bool en)
{
    _pa_nlow = en; commit_pa_state();
}

void umtrx_impl::set_diversity(bool en, int chan)
{
    _iface->poke32(U2_REG_SR_ADDR(SR_DIVSW + chan), (en != (chan == 1)) ? 0 : 1);
}

const char* umtrx_impl::get_hw_rev() const
{
    switch (_hw_rev) {
    case UMTRX_VER_2_0:    return "2.0";
    case UMTRX_VER_2_1:    return "2.1";
    case UMTRX_VER_2_2:    return "2.2";
    case UMTRX_VER_2_3_0:  return "2.3.0";
    case UMTRX_VER_2_3_1:  return "2.3.1";
    default:               return "[unknown]";
    }
}

void umtrx_impl::detect_hw_dcdc_ver(const uhd::fs_path&)
{
    _hw_dcdc_ver = DCDC_VER_2_3_1_OLD;
    if (_hw_rev < UMTRX_VER_2_3_1)
    {
        return;
    }

    uint8_t old = _pa_dcdc_r;
    bool old_pa_nlow = _pa_nlow;

    set_nlow(true);
    set_pa_dcdc_r(0);
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    double v_actual = read_dc_v("DCOUT").to_real();

    double err_min = std::abs(v_actual - _dcdc_val_to_volt[0][0]);
    for (unsigned j = 1; j < DCDC_VER_COUNT; ++j) {
        double err = std::abs(v_actual - _dcdc_val_to_volt[j][0]);
        if (err < err_min) {
            err_min = err;
            _hw_dcdc_ver = j;
        }
    }
    set_pa_dcdc_r(old);
    set_nlow(old_pa_nlow);

    UHD_LOGGER_INFO("UMTRX") << "Detected UmTRX DCDC ver. " << _hw_dcdc_ver
        << " (err: " << err_min << ")";
}

uhd::gain_range_t umtrx_impl::generate_tx_power_range(const std::string& which) const
{
    uhd::gain_range_t pa_range = generate_pa_power_range(which);
    uhd::gain_range_t vga_range(pa_range.start() - (_umtrx_vga2_def - UMTRX_VGA2_MIN), pa_range.start() - 1, 1.0);
    uhd::gain_range_t res_range(vga_range);
    res_range.insert(res_range.end(), pa_range.begin(), pa_range.end());
    return res_range;
}