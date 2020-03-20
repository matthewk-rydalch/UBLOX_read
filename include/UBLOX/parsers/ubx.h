#ifndef UBX_H
#define UBX_H

#include <stdint.h>
#include <iostream>

#include "async_comm/serial.h"
#include "UBLOX/parsers/ubx_defs.h"

namespace ublox
{

class UBX
{
public:

    UBX(async_comm::Serial& ser);

    void configure(uint8_t version, uint8_t layer, uint64_t cfgData, uint32_t cfgDataKey, uint8_t size);
    void get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    void del_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);

    // This function returns true when a new message has been parsed
    bool read_cb(uint8_t byte);

    // returns true if there is new data that hasn't been read
    // subsequent calls to new_data will return false after the
    // first time after a new message
    bool new_data();

    // callback handling
    typedef std::function<void(uint8_t, uint8_t, const UBX_message_t&)> ubx_cb;
    struct callback_t
    {
        uint8_t cls;
        uint8_t type;
        ubx_cb cb;
    };
    void registerCallback(uint8_t cls, uint8_t type, ubx_cb cb);
    std::vector<callback_t> callbacks;

    bool parsing_message();

    size_t num_messages_received();

    void set_nav_rate(uint16_t period_ms);

    // Send the supplied message
    bool send_message(uint8_t msg_class, uint8_t msg_id,
                      UBX_message_t& message, uint16_t len);

    // Main buffers for communication
    UBX_message_t out_message_;
    UBX_message_t in_message_;

    // low-level parsing functions
    bool decode_message();
    void calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id,
                            const uint16_t len, const UBX_message_t payload,
                            uint8_t &ck_a, uint8_t &ck_b) const;

    // Parsing State Working Memory
    uint8_t prev_byte_;
    uint16_t buffer_head_ = 0;
    bool start_message_ = false;
    bool end_message_ = false;
    bool got_ack_ = false;
    bool got_nack_ = false;
    parse_state_t parse_state_;
    uint8_t message_class_;
    uint8_t message_type_;
    uint16_t length_;
    uint8_t ck_a_;
    uint8_t ck_b_;
    uint32_t num_errors_ = 0;
    uint32_t num_messages_received_ = 0;
    uint8_t version;  //0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint32_t cfgDataKey;
    uint64_t cfgData;
    uint8_t size;
    uint8_t byte = 1;
    uint8_t word = 2;
    static std::map<uint8_t, std::string> ACK_msg_map;
    static std::map<uint8_t, std::string> AID_msg_map;
    static std::map<uint8_t, std::string> CFG_msg_map;
    static std::map<uint8_t, std::string> INF_msg_map;
    static std::map<uint8_t, std::string> LOG_msg_map;
    static std::map<uint8_t, std::string> MGA_msg_map;
    static std::map<uint8_t, std::string> MON_msg_map;
    static std::map<uint8_t, std::string> NAV_msg_map;
    static std::map<uint8_t, std::string> RXM_msg_map;
    static std::map<uint8_t, std::string> SEC_msg_map;
    static std::map<uint8_t, std::string> TIM_msg_map;
    static std::map<uint8_t, std::string> UPD_msg_map;
    static std::map<uint8_t, std::map<uint8_t, std::string>> UBX_map;
    
    // local storage
    volatile bool new_data_;

    // Serial Port
    async_comm::Serial& serial_;
};

}
#endif // UBX_H




