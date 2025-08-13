#include "apply_corrections.hpp"
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/utils/paths.hpp>
#include <boost/version.hpp>
#include "umtrx_log_adapter.hpp"
#include <uhd/utils/csv.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/version.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <cstdio>
#include <complex>
#include <fstream>

namespace fs = boost::filesystem;

boost::mutex corrections_mutex;

static double linear_interp(double x, double x0, double y0, double x1, double y1) {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

struct fe_cal_t {
    double lo_freq;
    double iq_corr_real;
    double iq_corr_imag;
};

static bool fe_cal_comp(fe_cal_t a, fe_cal_t b) {
    return (a.lo_freq < b.lo_freq);
}

static uhd::dict<std::string, std::vector<fe_cal_t> > fe_cal_cache;

static bool is_same_freq(const double f1, const double f2)
{
    const double epsilon = 0.1;
    return ((f1 - epsilon) < f2 and (f1 + epsilon) > f2);
}

static std::complex<double> get_fe_correction(
    const std::string& key, const double lo_freq
) {
    const std::vector<fe_cal_t>& datas = fe_cal_cache[key];
    if (datas.empty()) throw uhd::runtime_error("empty calibration table " + key);

    size_t lo_index = 0;
    size_t hi_index = datas.size() - 1;
    for (size_t i = 0; i < datas.size(); i++) {
        if (is_same_freq(datas[i].lo_freq, lo_freq))
        {
            hi_index = i;
            lo_index = i;
            break;
        }
        if (datas[i].lo_freq > lo_freq) {
            hi_index = i;
            break;
        }
        lo_index = i;
    }

    if (lo_index == 0) return std::complex<double>(datas[lo_index].iq_corr_real, datas[lo_index].iq_corr_imag);
    if (hi_index == lo_index) return std::complex<double>(datas[hi_index].iq_corr_real, datas[hi_index].iq_corr_imag);

    return std::complex<double>(
        linear_interp(lo_freq, datas[lo_index].lo_freq, datas[lo_index].iq_corr_real, datas[hi_index].lo_freq, datas[hi_index].iq_corr_real),
        linear_interp(lo_freq, datas[lo_index].lo_freq, datas[lo_index].iq_corr_imag, datas[hi_index].lo_freq, datas[hi_index].iq_corr_imag)
    );
}

static void apply_fe_corrections(
    uhd::property_tree::sptr sub_tree,
    const uhd::fs_path& db_path,
    const uhd::fs_path& fe_path,
    const std::string& file_prefix,
    const double lo_freq
) {
    const uhd::usrp::dboard_eeprom_t db_eeprom = sub_tree->access<uhd::usrp::dboard_eeprom_t>(db_path).get();

#if UHD_VERSION >= 4000000
    const fs::path cal_data_path = fs::path(uhd::get_cal_data_path()) / (file_prefix + db_eeprom.serial + ".csv");
#else
    const fs::path cal_data_path = fs::path(uhd::get_app_path()) / ".uhd" / "cal" / (file_prefix + db_eeprom.serial + ".csv");
#endif
    UHD_LOGGER_INFO("UMTRX") << "Looking for FE correction at: " << cal_data_path.string() << "...";
    if (not fs::exists(cal_data_path)) {
        UHD_LOGGER_INFO("UMTRX") << "Not found";
        return;
    }

    UHD_LOGGER_INFO("UMTRX") << "Found, loading...";

    if (not fe_cal_cache.has_key(cal_data_path.string())) {
        std::ifstream cal_data(cal_data_path.string().c_str());
        const uhd::csv::rows_type rows = uhd::csv::to_rows(cal_data);

        bool read_data = false, skip_next = false;
        std::vector<fe_cal_t> datas;
        BOOST_FOREACH(const uhd::csv::row_type & row, rows) {
            if (not read_data and not row.empty() and row[0] == "DATA STARTS HERE") {
                read_data = true;
                skip_next = true;
                continue;
            }
            if (not read_data) continue;
            if (skip_next) {
                skip_next = false;
                continue;
            }
            fe_cal_t data;
            std::sscanf(row[0].c_str(), "%lf", &data.lo_freq);
            std::sscanf(row[1].c_str(), "%lf", &data.iq_corr_real);
            std::sscanf(row[2].c_str(), "%lf", &data.iq_corr_imag);
            datas.push_back(data);
        }
        std::sort(datas.begin(), datas.end(), fe_cal_comp);
        fe_cal_cache[cal_data_path.string()] = datas;
        UHD_LOGGER_INFO("UMTRX") << "Loaded";
    }
    else {
        UHD_LOGGER_INFO("UMTRX") << "Loaded from cache";
    }

    sub_tree->access<std::complex<double> >(fe_path)
        .set(get_fe_correction(cal_data_path.string(), lo_freq));
}

void uhd::usrp::apply_tx_fe_corrections(
    property_tree::sptr sub_tree,
    const std::string& slot,
    const double lo_freq
) {
    boost::mutex::scoped_lock l(corrections_mutex);
    try {
        apply_fe_corrections(
            sub_tree,
            "dboards/" + slot + "/tx_eeprom",
            "tx_frontends/" + slot + "/iq_balance/value",
            "tx_iq_cal_v0.2_",
            lo_freq
        );
        apply_fe_corrections(
            sub_tree,
            "dboards/" + slot + "/tx_eeprom",
            "tx_frontends/" + slot + "/dc_offset/value",
            "tx_dc_cal_v0.2_",
            lo_freq
        );
    }
    catch (const std::exception& e) {
        UHD_LOGGER_ERROR("UMTRX") << "Failure in apply_tx_fe_corrections: " << e.what();
    }
}

void uhd::usrp::apply_rx_fe_corrections(
    property_tree::sptr sub_tree,
    const std::string& slot,
    const double lo_freq
) {
    boost::mutex::scoped_lock l(corrections_mutex);
    try {
        apply_fe_corrections(
            sub_tree,
            "dboards/" + slot + "/rx_eeprom",
            "rx_frontends/" + slot + "/iq_balance/value",
            "rx_iq_cal_v0.2_",
            lo_freq
        );
    }
    catch (const std::exception& e) {
        UHD_LOGGER_ERROR("UMTRX") << "Failure in apply_rx_fe_corrections: " << e.what();
    }
}