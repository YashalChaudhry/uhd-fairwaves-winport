#include "usrp2/fw_common.h"
#include "umtrx_iface.hpp"
#include "umtrx_log_adapter.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/asio.hpp>
#include <boost/foreach.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

device_addrs_t umtrx_find(const device_addr_t& hint) {

    device_addrs_t umtrx_addrs;

    if (hint.has_key("resource")) return umtrx_addrs;

    if (hint.has_key("type") and hint["type"] != "umtrx") return umtrx_addrs;

    if (not hint.has_key("addr")) {
        BOOST_FOREACH(const if_addrs_t & if_addrs, get_if_addrs()) {
            if (if_addrs.inet == asio::ip::address_v4::loopback().to_string()) continue;

            device_addr_t new_hint = hint;
            new_hint["addr"] = if_addrs.bcast;

            device_addrs_t new_umtrx_addrs = umtrx_find(new_hint);
            umtrx_addrs.insert(umtrx_addrs.begin(),
                new_umtrx_addrs.begin(), new_umtrx_addrs.end()
            );
        }
        return umtrx_addrs;
    }

    udp_simple::sptr udp_transport;
    try {
        udp_transport = udp_simple::make_broadcast(hint["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT));
    }
    catch (const std::exception& e) {
        UHD_LOGGER_ERROR("UMTRX") << "Cannot open UDP transport on " << hint["addr"] << "\n" << e.what();
        return umtrx_addrs;
    }

    usrp2_ctrl_data_t ctrl_data_out = usrp2_ctrl_data_t();
    ctrl_data_out.proto_ver = uhd::htonx<boost::uint32_t>(USRP2_FW_COMPAT_NUM);
    ctrl_data_out.id = uhd::htonx<boost::uint32_t>(UMTRX_CTRL_ID_REQUEST);
    try
    {
        udp_transport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
    }
    catch (const std::exception& ex)
    {
        UHD_LOGGER_ERROR("UMTRX") << "UmTRX Network discovery error " << ex.what();
    }
    catch (...)
    {
        UHD_LOGGER_ERROR("UMTRX") << "UmTRX Network discovery unknown error";
    }

    boost::uint8_t usrp2_ctrl_data_in_mem[udp_simple::mtu];
    const usrp2_ctrl_data_t* ctrl_data_in = reinterpret_cast<const usrp2_ctrl_data_t*>(usrp2_ctrl_data_in_mem);
    while (true) {
        size_t len = udp_transport->recv(asio::buffer(usrp2_ctrl_data_in_mem));
        if (len > offsetof(usrp2_ctrl_data_t, data) and ntohl(ctrl_data_in->id) == UMTRX_CTRL_ID_RESPONSE) {

            device_addr_t new_addr;
            new_addr["type"] = "umtrx";
            new_addr["addr"] = udp_transport->get_recv_addr();

            udp_simple::sptr ctrl_xport = udp_simple::make_connected(
                new_addr["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
            );
            ctrl_xport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
            size_t len = ctrl_xport->recv(asio::buffer(usrp2_ctrl_data_in_mem));
            if (len > offsetof(usrp2_ctrl_data_t, data) and ntohl(ctrl_data_in->id) == UMTRX_CTRL_ID_RESPONSE) {
            }
            else {
                continue;
            }

            try {
                umtrx_iface::sptr iface = umtrx_iface::make(ctrl_xport);
                if (iface->is_device_locked()) continue;
                mboard_eeprom_t mb_eeprom = iface->mb_eeprom;
                new_addr["name"] = mb_eeprom["name"];
                new_addr["serial"] = mb_eeprom["serial"];
            }
            catch (const std::exception&) {
                new_addr["name"] = "";
                new_addr["serial"] = "";
            }

            if (
                (not hint.has_key("name") or hint["name"] == new_addr["name"]) and
                (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
                ) {
                umtrx_addrs.push_back(new_addr);
            }
        }
        if (len == 0) break;
    }

    return umtrx_addrs;
}