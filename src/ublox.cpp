#include "UBLOX/ublox.h"
#include <typeinfo>
#define DBG(...) fprintf(stderr, __VA_ARGS__)

namespace ublox
{

UBLOX::UBLOX(const std::string& port) :
    serial_(port, 460800),
    //115200
    ubx_(serial_)
{
    type_ = NONE;

    auto cb = [this](const uint8_t* buffer, size_t size)
    {
        this->serial_read_cb(buffer, size);
    };
    serial_.register_receive_callback(cb);
    serial_.init();

    // auto eph_cb = [this](uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg)
    // {
    //   this->nav_.convertUBX(in_msg.RXM_SFRBX);
    // };
    // ubx_.registerCallback(ublox::CLASS_RXM, ublox::RXM_SFRBX, eph_cb);
}

void UBLOX::config_f9p() //See ubx_defs.h for more information
{
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, CFG_VALSET_t::DYNMODE_AIRBORNE_1G, CFG_VALSET_t::DYNMODEL, byte); //Dynamic platform model
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::USB_INPROT_NMEA, byte); //Flag to indicate if NMEA should be an input protocol on USB
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::USB_OUTPROT_NMEA, byte); //Flag to indicate if NMEA should be an output protocol on USB

    bool poll = false;
    if(poll == true)
        poll_value();
}

/*
Function config_base
Configures whether the base is stationary or moving.
Inputs: base type
Outputs: calls the correct base configuration function and outputs to the command line the status of the base.
*/
void UBLOX::config_base(std::string base_type, int gps, int glonas, int beidou,
                  int galileo, int surveytime, int surveyacc)
{
    std::cerr<<"Configuring Base\n";
    //Choose to configure as moving/mobile base or stationary
    //bool mobile = false;
    if(base_type == "moving")   //Moving base
    {
        config_base_moving(1, gps, glonas, beidou, galileo);
        config_base_stationary(0, gps, glonas, beidou, galileo, surveytime, surveyacc);
        std::cerr<<"Moving Base\n";
    }
    else if(base_type == "stationary")  //Stationary base
    {
        config_base_moving(0, gps, glonas, beidou, galileo);
        config_base_stationary(1, gps, glonas, beidou, galileo, surveytime, surveyacc);
        std::cerr<<"Stationary Base\n";
    }
    else    //Error thrown when type of base is unrecognized
    {
        throw std::runtime_error("Failed to initialize base as moving or stationary");
    }
}

void UBLOX::config_base_stationary(int on_off, int gps, int glonas, int beidou,
                  int galileo, int surveytime, int surveyacc)
{
    // ubx_.del_configuration(CFG_VALDEL_t::VERSION_0, CFG_VALDEL_t::RAM, CFG_VALDEL_t::TMODE_SVIN_MIN_DUR);
    // These values control whether RTK corrections are calculated for the
    // following constellations
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1005USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*gps, CFG_VALSET_t::RTCM_1074USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*glonas, CFG_VALSET_t::RTCM_1084USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*galileo, CFG_VALSET_t::RTCM_1094USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*beidou, CFG_VALSET_t::RTCM_1124USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*glonas, CFG_VALSET_t::RTCM_1230USB, byte);

    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::MSGOUT_SVIN, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::TMODE_MODE, byte);

    // Survey in accuracy limit
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, surveyacc*on_off, CFG_VALSET_t::TMODE_SVIN_ACC_LIMIT, word);
    // Survey in time limit
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, surveytime*on_off, CFG_VALSET_t::TMODE_SVIN_MIN_DUR, word);

}

void UBLOX::config_base_moving(int on_off, int gps, int glonas, int beidou,
                  int galileo)
{
    // These values control whether RTK corrections are calculated for the
    // following constellations
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_4072_0USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_4072_1USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*gps, CFG_VALSET_t::RTCM_1077USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*glonas, CFG_VALSET_t::RTCM_1087USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*galileo, CFG_VALSET_t::RTCM_1097USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off*beidou, CFG_VALSET_t::RTCM_1127USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*glonas, CFG_VALSET_t::RTCM_1230USB, byte);
}

void UBLOX::config_rover()
{
    // configure the parsers/Enable Messages
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_PVT, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_RELPOSNED, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_POSECEF, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_VELECEF, byte);
    // ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::MSGOUT_RAWX, byte);
    // ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::MSGOUT_SFRBX, byte);
    //configuring SVIN messages is done in config_base_stationary()
}

void UBLOX::poll_value() //See ubx_defs.h for more information
{
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1005USB); //CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Reference Station ARP
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1230USB); //CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2 Code-Phase Biases
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1097USB); //CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7 (high precision)
}

void UBLOX::readFile(const std::string& filename)
{
    std::ifstream file(filename,  std::ifstream::binary);
    file.seekg(0, file.end);
    uint32_t len = file.tellg();
    file.seekg (0, file.beg);
    char* buffer = new char [len];
    file.read(buffer, len);
    int idx = 0;
    while(idx < len)
    {
        int chunk_size = ((len - idx) >= 14)?14:len-idx;
        serial_read_cb(((const uint8_t*)buffer)+idx, chunk_size);
        idx = idx+chunk_size;
        usleep(1000);
    }
    // serial_read_cb((const uint8_t*)buffer, len);
}

void UBLOX::initLogFile(const std::string& filename)
{
    if (log_file_.is_open())
        log_file_.close();

    log_file_.open(filename);
}

// Function initRover
// This function initializes a rover that is given by the following data.
void UBLOX::initRover(std::string local_host, uint16_t local_port,
                      std::string remote_host, uint16_t remote_port)
{
    std::cerr << "initRover \n";
    type_ = ROVER;

    assert(udp_ == nullptr);
    // Connect the rtcm_cb callback to forward data to the UBX serial port
    rtcm_.registerCallback([this](uint8_t* buf, size_t size)
    {
        this->rtcm_complete_cb(buf, size);
    });

    // hook up UDP to listen
    /// TODO: configure ports and IP from cli
    udp_ = new async_comm::UDP(local_host, local_port, remote_host, remote_port);
    udp_->register_receive_callback([this](const uint8_t* buf, size_t size)
    {
        this->udp_read_cb(buf, size);
    });

    if (!udp_->init())
        throw std::runtime_error("Failed to initialize Rover receive UDP");

    config_rover();
    config_f9p();
    std::cerr<<"Initialized Rover\n";
}

/*
  initBase function
  Purpose: Initializes the base to send corrections to rovers
  Inputs: local_host[] an array of strings containing the local hosts for base. (remote hose for rover)
          local_port[] an array of uint16_t that contains local port numbers for base. (remote ports for rover)
          remote_host[] an array of strings containing the remote hosts for base (local host for rover)
          remote_port[] an array of uint16_t that contains remote ports for base. (local ports for rover)
          base_type stationary or moving base
          rover_quantity number of rovers (number of elements in each array)
*/
void UBLOX::initBase(std::string local_host[], uint16_t local_port[],
                std::string remote_host[], uint16_t remote_port[],
                std::string base_type, int rover_quantity, int gps,
                int glonas, int beidou, int galileo, int surveytime,
                int surveyacc)
{
    std::cerr << "initBase \n";
    type_ = BASE;

    //Instantiate an array of UDP objects
    if(rover_quantity>0)
        udparray_ = new async_comm::UDP*[rover_quantity];
    else
    {
            udparray_=NULL;
    }
    

    //Fill udp objects into the array.
    for(int i = 0; i < rover_quantity; i++) {
        std::cerr<<"Initializing Base to Rover "<<std::to_string(i+1)<<" UDP\n";

        //Create pointer to UDP object within an array
        udparray_[i] = new async_comm::UDP(local_host[i], local_port[i], remote_host[i], remote_port[i]);

        if (!udparray_[i]->init())
        {
            throw std::runtime_error("Failed to initialize Base to Rover "+ std::to_string(i+1) +" receive UDP\n");
        }

        rtcm_.registerCallback([ this , i](uint8_t* buf, size_t size)
        {
            this->udparray_[i]->send_bytes(buf, size);
        });

        std::cerr<<"Initialized Base to Rover "+ std::to_string(i+1) +" UDP\n";
    }

    config_base(base_type, gps, glonas, beidou, galileo, surveytime, surveyacc);
    config_f9p();
}

void UBLOX::initBrover(std::string local_host[], uint16_t local_port[],
                std::string base_host[], uint16_t base_port[],
                std::string rover_host[], uint16_t rover_port[],
                std::string base_type, int rover_quantity, int gps,
                int glonas, int beidou, int galileo) {

                  type_ = BROVER;

                  assert(udp_ == nullptr);
                  // Connect the rtcm_cb callback to forward data to the UBX serial port
                  rtcm_.registerCallback([this](uint8_t* buf, size_t size)
                  {
                      this->rtcm_complete_cb(buf, size);
                  });

                  // hook up UDP to listen
                  /// TODO: configure ports and IP from cli
                  udp_ = new async_comm::UDP(local_host[0], local_port[0], base_host[0], base_port[0]);
                  udp_->register_receive_callback([this](const uint8_t* buf, size_t size)
                  {
                      this->udp_read_cb(buf, size);
                  });

                  if (!udp_->init())
                      throw std::runtime_error("Failed to initialize Brover receive UDP");

                  config_rover();

                  //Instantiate an array of UDP objects
                  udparray_ = new async_comm::UDP*[rover_quantity];
                  std::cerr<<"Rover Quantity: "<<rover_quantity<<"\n";

                  //Fill udp objects into the array.
                  for(int i = 0; i < rover_quantity; i++) {
                      std::cerr<<"Initializing Brover at "<< local_host[i+1]<<", "<<local_port[i+1]
                      <<" to Rover "<<std::to_string(i+1)<<" at "<< rover_host[i]<<", "<<rover_port[i]<<" UDP\n";

                      //Create pointer to UDP object within an array
                      udparray_[i] = new async_comm::UDP(local_host[i+1], local_port[i+1], rover_host[i], rover_port[i]);

                      if (!udparray_[i]->init())
                      {
                          throw std::runtime_error("Failed to initialize Brover to Rover "+ std::to_string(i+1) +"  UDP\n");
                      }

                      rtcm_.registerCallback([ this , i](uint8_t* buf, size_t size)
                      {
                          this->udparray_[i]->send_bytes(buf, size);
                      });

                      std::cerr<<"Initialized Brover to Rover "+ std::to_string(i+1) +" UDP\n";
                  }

                  config_base(base_type, gps, glonas, beidou, galileo, 0, 0);

                  config_f9p();
                  std::cerr<<"Initialized Brover\n";
}

UBLOX::~UBLOX()
{
    if (udp_)
        delete udp_;

    if (log_file_.is_open())
        log_file_.close();
}

void UBLOX::udp_read_cb(const uint8_t* buf, size_t size)
{

    assert(type_ == ROVER || type_ == BROVER);
    for (int i = 0; i < size; i++)
    {
        rtcm_.read_cb(buf[i]);
    }
}

void UBLOX::serial_read_cb(const uint8_t *buf, size_t size)
{

    for (int i = 0; i < size; i++)
    {
        /// TODO: don't give parsers data they don't need
        if (ubx_.parsing_message())
        {
            ubx_.read_cb(buf[i]);
        }
        else if (rtcm_.parsing_message() && type_ != NONE)
        {
            rtcm_.read_cb(buf[i]);
        }
        else
        {
            ubx_.read_cb(buf[i]);
            rtcm_.read_cb(buf[i]);
        }
    }

    if (log_file_.is_open())
    {
        log_file_.write((const char*)buf, size);
    }
}

void UBLOX::rtcm_complete_cb(const uint8_t *buf, size_t size)
{
    assert (type_ == ROVER || type_ == BASE || type_ == BROVER);
    if (type_ == ROVER)
        serial_.send_bytes(buf, size);
    else if (type_ == BASE)
        udp_->send_bytes(buf, size);
    else if (type_ == BROVER) {
        serial_.send_bytes(buf, size);
      }

}
}
