#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/types/mac_addr.hpp>
#include <uhd/types/byte_vector.hpp>
#include <uhd/utils/byteswap.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <algorithm>
#include <iostream>
#include <cstddef>

using namespace uhd;
using namespace uhd::usrp;

static const boost::uint8_t N200_EEPROM_ADDR = 0x50;
static const size_t SERIAL_LEN = 9;
static const size_t NAME_MAX_LEN = 32 - SERIAL_LEN;

static byte_vector_t string_to_uint16_bytes(const std::string& num_str) {
    const boost::uint16_t num = boost::lexical_cast<boost::uint16_t>(num_str);
    const byte_vector_t lsb_msb = boost::assign::list_of
    (boost::uint8_t(num >> 0))(boost::uint8_t(num >> 8));
    return lsb_msb;
}

static std::string uint16_bytes_to_string(const byte_vector_t& bytes) {
    const boost::uint16_t num = (boost::uint16_t(bytes.at(0)) << 0) | (boost::uint16_t(bytes.at(1)) << 8);
    return (num == 0 or num == 0xffff) ? "" : boost::lexical_cast<std::string>(num);
}

struct n200_eeprom_map {
    uint16_t hardware;
    uint8_t mac_addr[6];
    uint32_t subnet;
    uint32_t ip_addr;
    uint16_t _pad0;
    uint16_t revision;
    uint16_t product;
    unsigned char _pad1;
    unsigned char gpsdo;
    unsigned char serial[SERIAL_LEN];
    unsigned char name[NAME_MAX_LEN];
    uint32_t gateway;
};

enum n200_gpsdo_type {
    N200_GPSDO_NONE = 0,
    N200_GPSDO_INTERNAL = 1,
    N200_GPSDO_ONBOARD = 2
};

static const uhd::dict<std::string, boost::uint8_t> UMTRX_OFFSETS = boost::assign::map_list_of
("tx1-vga1-dc-i", 0xFF - 0)
("tx1-vga1-dc-q", 0xFF - 1)
("tcxo-dac", 0xFF - 3)
("tx2-vga1-dc-i", 0xFF - 4)
("tx2-vga1-dc-q", 0xFF - 5)
("pa_dcdc_r", 0xFF - 6)
("pa_low", 0xFF - 7)
("pa_en1", 0xFF - 8)
("pa_en2", 0xFF - 9)
;

#if 0x18 + SERIAL_LEN + NAME_MAX_LEN >= 0xFF-7
#   error EEPROM address overlap! Get a bigger EEPROM.
#endif

void load_umtrx_eeprom(mboard_eeprom_t& mb_eeprom, i2c_iface& iface) {
    mb_eeprom = mboard_eeprom_t();

    mb_eeprom["hardware"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, hardware), 2)
    );

    mb_eeprom["revision"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, revision), 2)
    );

    mb_eeprom["product"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, product), 2)
    );

    mb_eeprom["mac-addr"] = mac_addr_t::from_bytes(iface.read_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, mac_addr), 6
    )).to_string();

    boost::asio::ip::address_v4::bytes_type ip_addr_bytes;
    byte_copy(iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, ip_addr), 4), ip_addr_bytes);
    mb_eeprom["ip-addr"] = boost::asio::ip::address_v4(ip_addr_bytes).to_string();

    byte_copy(iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, subnet), 4), ip_addr_bytes);
    mb_eeprom["subnet"] = boost::asio::ip::address_v4(ip_addr_bytes).to_string();

    byte_copy(iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gateway), 4), ip_addr_bytes);
    mb_eeprom["gateway"] = boost::asio::ip::address_v4(ip_addr_bytes).to_string();

    uint8_t gpsdo_byte = iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gpsdo), 1).at(0);
    switch (n200_gpsdo_type(gpsdo_byte)) {
    case N200_GPSDO_INTERNAL: mb_eeprom["gpsdo"] = "internal"; break;
    case N200_GPSDO_ONBOARD: mb_eeprom["gpsdo"] = "onboard"; break;
    default: mb_eeprom["gpsdo"] = "none";
    }

    mb_eeprom["serial"] = bytes_to_string(iface.read_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, serial), SERIAL_LEN
    ));

    mb_eeprom["name"] = bytes_to_string(iface.read_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, name), NAME_MAX_LEN
    ));

    if (mb_eeprom["serial"].empty()) {
        byte_vector_t mac_addr_bytes = mac_addr_t::from_string(mb_eeprom["mac-addr"]).to_bytes();
        unsigned serial = mac_addr_bytes.at(5) | (unsigned(mac_addr_bytes.at(4) & 0x0f) << 8);
        mb_eeprom["serial"] = std::to_string(serial);
    }

    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-i"], 1).at(0));
        mb_eeprom["tx1-vga1-dc-i"] = (val == 255) ? "" : boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-q"], 1).at(0));
        mb_eeprom["tx1-vga1-dc-q"] = (val == 255) ? "" : boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-i"], 1).at(0));
        mb_eeprom["tx2-vga1-dc-i"] = (val == 255) ? "" : boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-q"], 1).at(0));
        mb_eeprom["tx2-vga1-dc-q"] = (val == 255) ? "" : boost::lexical_cast<std::string>(int(val));
    }

    mb_eeprom["tcxo-dac"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tcxo-dac"], 2)
    );

    mb_eeprom["pa_dcdc_r"] =
        boost::lexical_cast<std::string>(unsigned(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_dcdc_r"], 1).at(0)));

    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_low"], 1).at(0));
        mb_eeprom["pa_low"] = (val == 255) ? "" : boost::lexical_cast<std::string>(int(val));
    }

    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en1"], 1).at(0));
        mb_eeprom["pa_en1"] = (val != 0) ? "1" : "0";
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en2"], 1).at(0));
        mb_eeprom["pa_en2"] = (val != 0) ? "1" : "0";
    }
}

void store_umtrx_eeprom(const mboard_eeprom_t& mb_eeprom, i2c_iface& iface)
{
    if (mb_eeprom.has_key("hardware")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, hardware),
        string_to_uint16_bytes(mb_eeprom["hardware"])
    );

    if (mb_eeprom.has_key("revision")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, revision),
        string_to_uint16_bytes(mb_eeprom["revision"])
    );

    if (mb_eeprom.has_key("product")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, product),
        string_to_uint16_bytes(mb_eeprom["product"])
    );

    if (mb_eeprom.has_key("mac-addr")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, mac_addr),
        mac_addr_t::from_string(mb_eeprom["mac-addr"]).to_bytes()
    );

    if (mb_eeprom.has_key("ip-addr")) {
        byte_vector_t ip_addr_bytes(4);
        byte_copy(boost::asio::ip::make_address_v4(mb_eeprom["ip-addr"]).to_bytes(), ip_addr_bytes);
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, ip_addr), ip_addr_bytes);
    }

    if (mb_eeprom.has_key("subnet")) {
        byte_vector_t ip_addr_bytes(4);
        byte_copy(boost::asio::ip::make_address_v4(mb_eeprom["subnet"]).to_bytes(), ip_addr_bytes);
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, subnet), ip_addr_bytes);
    }

    if (mb_eeprom.has_key("gateway")) {
        byte_vector_t ip_addr_bytes(4);
        byte_copy(boost::asio::ip::make_address_v4(mb_eeprom["gateway"]).to_bytes(), ip_addr_bytes);
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gateway), ip_addr_bytes);
    }

    if (mb_eeprom.has_key("gpsdo")) {
        uint8_t gpsdo_byte = N200_GPSDO_NONE;
        if (mb_eeprom["gpsdo"] == "internal") gpsdo_byte = N200_GPSDO_INTERNAL;
        if (mb_eeprom["gpsdo"] == "onboard") gpsdo_byte = N200_GPSDO_ONBOARD;
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gpsdo), byte_vector_t(1, gpsdo_byte));
    }

    if (mb_eeprom.has_key("serial")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, serial),
        string_to_bytes(mb_eeprom["serial"], SERIAL_LEN)
    );

    if (mb_eeprom.has_key("name")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, name),
        string_to_bytes(mb_eeprom["name"], NAME_MAX_LEN)
    );

    if (mb_eeprom.has_key("tx1-vga1-dc-i")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-i"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx1-vga1-dc-i"]))
    );
    if (mb_eeprom.has_key("tx1-vga1-dc-q")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-q"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx1-vga1-dc-q"]))
    );
    if (mb_eeprom.has_key("tx2-vga1-dc-i")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-i"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx2-vga1-dc-i"]))
    );
    if (mb_eeprom.has_key("tx2-vga1-dc-q")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-q"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx2-vga1-dc-q"]))
    );

    if (mb_eeprom.has_key("tcxo-dac")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tcxo-dac"],
        string_to_uint16_bytes(mb_eeprom["tcxo-dac"])
    );

    if (mb_eeprom.has_key("pa_dcdc_r")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_dcdc_r"],
        byte_vector_t(1, boost::lexical_cast<unsigned>(mb_eeprom["pa_dcdc_r"]))
    );

    if (mb_eeprom.has_key("pa_low")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_low"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_low"]))
    );

    if (mb_eeprom.has_key("pa_en1")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en1"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_en1"]))
    );

    if (mb_eeprom.has_key("pa_en2")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en2"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_en2"]))
    );
}