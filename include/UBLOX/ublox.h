#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "async_comm/serial.h"
#include "async_comm/udp.h"

#include "UBLOX/parsers/ubx.h"
#include "UBLOX/parsers/rtcm.h"
#include "UBLOX/parsers/nav.h"

namespace ublox
{

class UBLOX
{
public:

    typedef enum
    {
        NONE = 0,
        ROVER = 0b10, // 2
        BASE = 0b00,  //0
        BROVER = 0b01,  // 1
        RTK = 0b10,
    } rtk_type_t;
    const double PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620;

    UBLOX(const std::string& port);
    ~UBLOX();

    // legacy initBase function
    void initBase(std::string local_host, uint16_t local_port,
                  std::string remote_host, uint16_t remote_port,
                  std::string base_type);

    void initBase(std::string local_host[], uint16_t local_port[],
                    std::string remote_host[], uint16_t remote_port[],
                    std::string base_type, int rover_quantity, int gps,
                    int glonas, int beidou, int galileo, int surveytime,
                    int surveyacc);

    void initBase(std::string local_host[], uint16_t local_port[],
                    std::string remote_host[], uint16_t remote_port[],
                    std::string base_type, int rover_quantity, int gps,
                    int glonas, int beidou, int galileo);

    // Current initBase function supports multiple rovers
    void initBase(std::string local_host[], uint16_t local_port[],
                    std::string remote_host[], uint16_t remote_port[],
                    std::string base_type, int rover_quantity);

    void initRover(std::string local_host, uint16_t local_port,
                   std::string remote_host, uint16_t remote_port,
                   uint32_t constellation[]);

     void initRover(std::string local_host, uint16_t local_port,
                    std::string remote_host, uint16_t remote_port);


    // base_type refers to whether the brover is stationary or moving
    void initBrover(std::string local_host[], uint16_t local_port[],
                    std::string base_host[], uint16_t base_port[],
                    std::string rover_host[], uint16_t rover_port[],
                    std::string base_type, int rover_quantity);

    void initBrover(std::string local_host[], uint16_t local_port[],
                    std::string base_host[], uint16_t base_port[],
                    std::string rover_host[], uint16_t rover_port[],
                    std::string base_type, int rover_quantity, int gps,
                    int glonas, int beidou, int galileo);

    void initLogFile(const std::string& filename);
    void readFile(const std::string& filename);

    // Array of pointers to UDP objects.
    async_comm::UDP** udparray_ = nullptr;

    //Legacy UDP object replace by udparray_
    async_comm::UDP* udp_ = nullptr;
    async_comm::Serial serial_;

    UBX ubx_;
    RTCM rtcm_;
    NavParser nav_;

    std::ofstream log_file_;

    inline void registerUBXCallback(uint8_t cls, uint8_t type, UBX::ubx_cb cb)
    {
        ubx_.registerCallback(cls, type, cb);
    }
    inline void registerEphCallback(const NavParser::eph_cb& cb)
    {
        nav_.registerCallback(cb);
    }
    inline void registerGephCallback(const NavParser::geph_cb& cb)
    {
        nav_.registerCallback(cb);
    }


    rtk_type_t type_;

    void serial_read_cb(const uint8_t* buf, size_t size);
    void udp_read_cb(const uint8_t *buf, size_t size);
    void rtcm_complete_cb(const uint8_t* buf, size_t size);

    void config_f9p();
    void config_rover();
    void config_base(std::string base_type, int gps, int glonas, int beidou,
                      int galileo, int surveytime, int surveyacc);
    void config_base_stationary(int on_off, int gps, int glonas, int beidou,
                      int galileo, int surveytime, int surveyacc);
    void config_base_moving(int on_off, int gps, int glonas, int beidou,
                      int galileo);
    void poll_value();

    void vector_math(double ned_1[], double ned_2[], double answer[]);

    CFG_VALGET_TUPLE_t cfgValGet(const CFG_VALGET_t &request);
    CFG_VAL_DBG_t cfgValDel(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    CFG_VAL_DBG_t cfgValSet(uint8_t version, uint8_t layer, uint64_t cfgData, uint32_t cfgDataKey, uint8_t size);

    uint8_t byte = 1;
    uint8_t word = 2;
    int on_off;
};
}

#endif
