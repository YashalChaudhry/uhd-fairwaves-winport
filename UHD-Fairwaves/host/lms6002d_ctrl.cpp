#include "lms6002d_ctrl.hpp"
#include "lms6002d.hpp"
#include "cores/adf4350_regs.hpp"

#include <uhd/utils/log.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/assert_has.hpp>
#include <uhd/utils/algorithm.hpp>
#include "umtrx_log_adapter.hpp"
#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <utility>
#include <cmath>
#include <cfloat>
#include <limits>

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

static const freq_range_t lms_freq_range(0.2325e9, 3.72e9);

static const freq_range_t lms_bandwidth_range = list_of
(uhd::range_t(2 * 0.5 * 1e6))
(uhd::range_t(2 * 0.75 * 1e6))
(uhd::range_t(2 * 0.875 * 1e6))
(uhd::range_t(2 * 1.25 * 1e6))
(uhd::range_t(2 * 1.375 * 1e6))
(uhd::range_t(2 * 1.5 * 1e6))
(uhd::range_t(2 * 1.92 * 1e6))
(uhd::range_t(2 * 2.5 * 1e6))
(uhd::range_t(2 * 2.75 * 1e6))
(uhd::range_t(2 * 3 * 1e6))
(uhd::range_t(2 * 3.5 * 1e6))
(uhd::range_t(2 * 4.375 * 1e6))
(uhd::range_t(2 * 5 * 1e6))
(uhd::range_t(2 * 6 * 1e6))
(uhd::range_t(2 * 7 * 1e6))
(uhd::range_t(2 * 10 * 1e6))
(uhd::range_t(2 * 14 * 1e6))
;

static const std::vector<std::string> lms_tx_antennas = list_of("TX0")("TX1")("TX2")("CAL");

static const std::vector<std::string> lms_rx_antennas = list_of("RX0")("RX1")("RX2")("RX3")("CAL");

static const uhd::dict<std::string, gain_range_t> lms_tx_gain_ranges = map_list_of
("VGA2", gain_range_t(0, 25, double(1.0)))
("VGA1", gain_range_t(-35, -4, double(1.0)))
;

static const uhd::dict<std::string, gain_range_t> lms_rx_gain_ranges = map_list_of
("VGA1", gain_range_t(list_of
(uhd::range_t(5.00))(uhd::range_t(5.07))(uhd::range_t(5.14))(uhd::range_t(5.21))(uhd::range_t(5.28))
(uhd::range_t(5.35))(uhd::range_t(5.42))(uhd::range_t(5.49))(uhd::range_t(5.57))(uhd::range_t(5.64))
(uhd::range_t(5.71))(uhd::range_t(5.79))(uhd::range_t(5.86))(uhd::range_t(5.94))(uhd::range_t(6.01))
(uhd::range_t(6.09))(uhd::range_t(6.17))(uhd::range_t(6.25))(uhd::range_t(6.33))(uhd::range_t(6.41))
(uhd::range_t(6.49))(uhd::range_t(6.57))(uhd::range_t(6.65))(uhd::range_t(6.74))(uhd::range_t(6.82))
(uhd::range_t(6.90))(uhd::range_t(6.99))(uhd::range_t(7.08))(uhd::range_t(7.16))(uhd::range_t(7.25))
(uhd::range_t(7.34))(uhd::range_t(7.43))(uhd::range_t(7.52))(uhd::range_t(7.61))(uhd::range_t(7.71))
(uhd::range_t(7.80))(uhd::range_t(7.90))(uhd::range_t(7.99))(uhd::range_t(8.09))(uhd::range_t(8.19))
(uhd::range_t(8.29))(uhd::range_t(8.39))(uhd::range_t(8.49))(uhd::range_t(8.59))(uhd::range_t(8.69))
(uhd::range_t(8.80))(uhd::range_t(8.91))(uhd::range_t(9.01))(uhd::range_t(9.12))(uhd::range_t(9.23))
(uhd::range_t(9.35))(uhd::range_t(9.46))(uhd::range_t(9.57))(uhd::range_t(9.69))(uhd::range_t(9.81))
(uhd::range_t(9.93))(uhd::range_t(10.05))(uhd::range_t(10.17))(uhd::range_t(10.30))(uhd::range_t(10.43))
(uhd::range_t(10.55))(uhd::range_t(10.69))(uhd::range_t(10.82))(uhd::range_t(10.95))(uhd::range_t(11.09))
(uhd::range_t(11.23))(uhd::range_t(11.37))(uhd::range_t(11.51))(uhd::range_t(11.66))(uhd::range_t(11.81))
(uhd::range_t(11.96))(uhd::range_t(12.11))(uhd::range_t(12.27))(uhd::range_t(12.43))(uhd::range_t(12.59))
(uhd::range_t(12.76))(uhd::range_t(12.92))(uhd::range_t(13.10))(uhd::range_t(13.27))(uhd::range_t(13.45))
(uhd::range_t(13.63))(uhd::range_t(13.82))(uhd::range_t(14.01))(uhd::range_t(14.21))(uhd::range_t(14.41))
(uhd::range_t(14.61))(uhd::range_t(14.82))(uhd::range_t(15.03))(uhd::range_t(15.25))(uhd::range_t(15.48))
(uhd::range_t(15.71))(uhd::range_t(15.95))(uhd::range_t(16.19))(uhd::range_t(16.45))(uhd::range_t(16.71))
(uhd::range_t(16.97))(uhd::range_t(17.25))(uhd::range_t(17.53))(uhd::range_t(17.83))(uhd::range_t(18.13))
(uhd::range_t(18.45))(uhd::range_t(18.78))(uhd::range_t(19.12))(uhd::range_t(19.47))(uhd::range_t(19.84))
(uhd::range_t(20.23))(uhd::range_t(20.63))(uhd::range_t(21.06))(uhd::range_t(21.50))(uhd::range_t(21.97))
(uhd::range_t(22.47))(uhd::range_t(22.99))(uhd::range_t(23.55))(uhd::range_t(24.15))(uhd::range_t(24.80))
(uhd::range_t(25.49))(uhd::range_t(26.25))(uhd::range_t(27.08))(uhd::range_t(27.99))(uhd::range_t(29.01))
(uhd::range_t(30.17))
))
("VGA2", gain_range_t(0, 30, double(3.0)))
;


static int verbosity = 0;

class umtrx_lms6002d_dev : public lms6002d_dev {
    uhd::spi_iface::sptr _spiface;
    const int _slaveno;
public:
    umtrx_lms6002d_dev(uhd::spi_iface::sptr spiface, const int slaveno) : _spiface(spiface), _slaveno(slaveno) {};

    virtual void write_reg(uint8_t addr, uint8_t data) {
        if (verbosity > 2) printf("umtrx_lms6002d_dev::write_reg(addr=0x%x, data=0x%x)\n", addr, data);
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        _spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if (addr > 127) return 0;
        uint8_t data = _spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, addr << 8, 16);
        if (verbosity > 2) printf("umtrx_lms6002d_dev::read_reg(addr=0x%x) data=0x%x\n", addr, data);
        return data;
    }
};

class lms6002d_ctrl_impl : public lms6002d_ctrl {
public:
    lms6002d_ctrl_impl(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate);

    double set_rx_freq(const double freq)
    {
        return this->set_freq(dboard_iface::UNIT_RX, freq);
    }

    double set_tx_freq(const double freq)
    {
        return this->set_freq(dboard_iface::UNIT_TX, freq);
    }

    bool set_rx_enabled(const bool enb)
    {
        return this->set_enabled(dboard_iface::UNIT_RX, enb);
    }

    bool set_tx_enabled(const bool enb)
    {
        return this->set_enabled(dboard_iface::UNIT_TX, enb);
    }

    uhd::sensor_value_t get_rx_pll_locked()
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return uhd::sensor_value_t("LO", lms.get_rx_pll_locked(), "locked", "unlocked");
    }

    uhd::sensor_value_t get_tx_pll_locked()
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return uhd::sensor_value_t("LO", lms.get_tx_pll_locked(), "locked", "unlocked");
    }

    uhd::freq_range_t get_rx_bw_range(void)
    {
        return lms_bandwidth_range;
    }

    uhd::freq_range_t get_tx_bw_range(void)
    {
        return lms_bandwidth_range;
    }

    uhd::freq_range_t get_rx_freq_range(void)
    {
        return lms_freq_range;
    }

    uhd::freq_range_t get_tx_freq_range(void)
    {
        return lms_freq_range;
    }

    std::vector<std::string> get_tx_antennas(void)
    {
        return lms_tx_antennas;
    }

    std::vector<std::string> get_rx_antennas(void)
    {
        return lms_rx_antennas;
    }

    std::vector<std::string> get_tx_gains(void)
    {
        return lms_tx_gain_ranges.keys();
    }

    std::vector<std::string> get_rx_gains(void)
    {
        return lms_rx_gain_ranges.keys();
    }

    uhd::gain_range_t get_rx_gain_range(const std::string& name)
    {
        return lms_rx_gain_ranges[name];
    }

    uhd::gain_range_t get_tx_gain_range(const std::string& name)
    {
        return lms_tx_gain_ranges[name];
    }

    uint8_t get_tx_vga1dc_i_int(void)
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return lms.get_tx_vga1dc_i_int();
    }

    uint8_t get_tx_vga1dc_q_int(void)
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return lms.get_tx_vga1dc_q_int();
    }

protected:

    double set_freq(dboard_iface::unit_t unit, double f) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_freq(%f)\n", f);
        unsigned ref_freq = _clock_rate;
        double actual_freq = 0;
        if (unit == dboard_iface::UNIT_TX) {
            actual_freq = lms.tx_pll_tune(ref_freq, f);
        }
        else if (unit == dboard_iface::UNIT_RX) {
            actual_freq = lms.rx_pll_tune(ref_freq, f);
        }
        else {
            assert(!"Wrong units_t value passed to lms6002d_ctrl_impl::set_freq()");
        }
        if (actual_freq < 0)
            actual_freq = 0;
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_freq() actual_freq=%f\n", actual_freq);
        return actual_freq;
    }

    bool set_enabled(dboard_iface::unit_t unit, bool en) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_enabled(%d)\n", en);
        if (unit == dboard_iface::UNIT_RX) {
            if (en)
                lms.rx_enable();
            else
                lms.rx_disable();
        }
        else {
            if (en)
                lms.tx_enable();
            else
                lms.tx_disable();
        }
        return en;
    }

    double set_rx_gain(double gain, const std::string& name) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rx_gain(%f, %s)\n", gain, name.c_str());
        assert_has(lms_rx_gain_ranges.keys(), name, "LMS6002D rx gain name");
        if (name == "VGA1") {
            lms.set_rx_vga1gain(gain);
            return lms.get_rx_vga1gain();
        }
        else if (name == "VGA2") {
            lms.set_rx_vga2gain(gain);
            return lms.get_rx_vga2gain();
        }
        else UHD_THROW_INVALID_CODE_PATH();
        return gain;
    }

    void set_rx_ant(const std::string& ant) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rx_ant(%s)\n", ant.c_str());
        assert_has(lms_rx_antennas, ant, "LMS6002D rx antenna name");

        if (ant == "CAL") {
            if (!rf_loopback_enabled) {
                if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rx_ant(%s) enabling RF loopback for LNA%d\n", ant.c_str(), lms.get_rx_lna());
                lms.rf_loopback_enable(lms.get_rx_lna());
                rf_loopback_enabled = true;
            }
        }
        else {
            if (rf_loopback_enabled) {
                lms.rf_loopback_disable();
                rf_loopback_enabled = false;
            }

            if (ant == "RX0") {
                lms.set_rx_lna(0);
            }
            else if (ant == "RX1") {
                lms.set_rx_lna(1);
            }
            else if (ant == "RX2") {
                lms.set_rx_lna(2);
            }
            else if (ant == "RX3") {
                lms.set_rx_lna(3);
            }
            else {
                UHD_THROW_INVALID_CODE_PATH();
            }
        }
    }

    double set_rx_bandwidth(double bandwidth) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rx_bandwidth(%f)\n", bandwidth);
        bandwidth = lms_bandwidth_range.clip(bandwidth);
        lms.set_rx_lpf(int(bandwidth / 2 / 1e3));
        return bandwidth;
    }

    double set_tx_gain(double gain, const std::string& name) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_tx_gain(%f, %s)\n", gain, name.c_str());
        assert_has(lms_tx_gain_ranges.keys(), name, "LMS6002D tx gain name");

        if (name == "VGA1") {
            if (verbosity > 1) printf("db_lms6002d::set_tx_gain() VGA1=%d\n", int(gain));
            lms.set_tx_vga1gain(-10);
            lms.set_tx_vga1gain(int(gain));
            lms.auto_calibration(_clock_rate, 0xf);
            return lms.get_tx_vga1gain();
        }
        else if (name == "VGA2") {
            if (verbosity > 1) printf("db_lms6002d::set_tx_gain() VGA2=%d\n", int(gain));
            lms.set_tx_vga2gain(int(gain));
            return lms.get_tx_vga2gain();
        }
        else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        return tx_vga1gain + tx_vga2gain;
    }

    void set_tx_ant(const std::string& ant) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_tx_ant(%s)\n", ant.c_str());
        assert_has(lms_tx_antennas, ant, "LMS6002D tx antenna ant");

        if (ant == "TX0") {
            lms.set_tx_pa(0);
        }
        else if (ant == "TX1") {
            lms.set_tx_pa(1);
        }
        else if (ant == "TX2") {
            lms.set_tx_pa(2);
        }
        else if (ant == "CAL") {
            lms.set_tx_pa(0);
        }
        else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    }

    double set_tx_bandwidth(double bandwidth) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_tx_bandwidth(%f)\n", bandwidth);
        bandwidth = lms_bandwidth_range.clip(bandwidth);
        lms.set_tx_lpf(int(bandwidth / 2 / 1e3));
        return bandwidth;
    }

    uint8_t _set_tx_vga1dc_i_int(uint8_t offset) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_tx_vga1dc_i_int(%d)\n", offset);
        lms.set_tx_vga1dc_i_int(offset);
        return offset;
    }

    uint8_t _set_tx_vga1dc_q_int(uint8_t offset) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_tx_vga1dc_q_int(%d)\n", offset);
        lms.set_tx_vga1dc_q_int(offset);
        return offset;
    }

    void set_rxfe_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxfe_dc_i(%d)\n", value);
        lms.set_rxfe_dc_i(value);
    }

    uint8_t get_rxfe_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxfe_dc_i()\n");
        return lms.get_rxfe_dc_i();
    }

    void set_rxfe_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxfe_dc_q(%d)\n", value);
        lms.set_rxfe_dc_q(value);
    }

    uint8_t get_rxfe_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxfe_dc_q()\n");
        return lms.get_rxfe_dc_q();
    }

    void set_rxlpf_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxlpf_dc_i(%d)\n", value);
        lms.set_rxlpf_dc_i(value);
    }

    uint8_t get_rxlpf_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxlpf_dc_i()\n");
        return lms.get_rxlpf_dc_i();
    }

    void set_rxlpf_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxlpf_dc_q(%d)\n", value);
        lms.set_rxlpf_dc_q(value);
    }

    uint8_t get_rxlpf_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxlpf_dc_q()\n");
        return lms.get_rxlpf_dc_q();
    }

    void set_rxvga2_dc_reference(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxvga2_dc_reference(%d)\n", value);
        lms.set_rxvga2_dc_reference(value);
    }

    uint8_t get_rxvga2_dc_reference() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxvga2_dc_reference()\n");
        return lms.get_rxvga2_dc_reference();
    }

    void set_rxvga2a_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxvga2a_dc_i(%d)\n", value);
        lms.set_rxvga2a_dc_i(value);
    }

    uint8_t get_rxvga2a_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxvga2a_dc_i()\n");
        return lms.get_rxvga2a_dc_i();
    }

    void set_rxvga2a_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxvga2a_dc_q(%d)\n", value);
        lms.set_rxvga2a_dc_q(value);
    }

    uint8_t get_rxvga2a_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxvga2a_dc_q()\n");
        return lms.get_rxvga2a_dc_q();
    }

    void set_rxvga2b_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxvga2b_dc_i(%d)\n", value);
        lms.set_rxvga2b_dc_i(value);
    }

    uint8_t get_rxvga2b_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxvga2b_dc_i()\n");
        return lms.get_rxvga2b_dc_i();
    }

    void set_rxvga2b_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::set_rxvga2b_dc_q(%d)\n", value);
        lms.set_rxvga2b_dc_q(value);
    }

    uint8_t get_rxvga2b_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity > 0) printf("lms6002d_ctrl_impl::get_rxvga2b_dc_q()\n");
        return lms.get_rxvga2b_dc_q();
    }

private:
    umtrx_lms6002d_dev lms;
    int tx_vga1gain, tx_vga2gain;
    bool rf_loopback_enabled;

    uhd::spi_iface::sptr _spiface;
    const int _lms_spi_number;
    const double _clock_rate;

    boost::recursive_mutex _mutex;
};

lms6002d_ctrl::sptr lms6002d_ctrl::make(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate)
{
    return sptr(new lms6002d_ctrl_impl(spiface, lms_spi_number, clock_rate));
}

lms6002d_ctrl_impl::lms6002d_ctrl_impl(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate) :
    lms(umtrx_lms6002d_dev(spiface, lms_spi_number)),
    tx_vga1gain(lms.get_tx_vga1gain()),
    tx_vga2gain(lms.get_tx_vga2gain()),
    rf_loopback_enabled(false),
    _spiface(spiface),
    _lms_spi_number(lms_spi_number),
    _clock_rate(clock_rate)
{
    lms.init();
    lms.set_txrx_polarity_and_interleaving(0, lms6002d_dev::INTERLEAVE_IQ, 1, lms6002d_dev::INTERLEAVE_QI);
    lms.rx_enable();
    lms.tx_enable();
    lms.set_tx_pa(0);
}