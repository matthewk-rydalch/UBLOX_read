#include <iostream>
#include <fstream>
#include <signal.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;


#include <UBLOX/ublox_ros.h>

#define createCallback(cls, type, fun, arg)\
do{\
    auto trampoline = [this](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg)\
    {\
        this->fun(msg.arg);\
    };\
    ublox_->registerUBXCallback(cls, type, trampoline);\
}while(0)

constexpr double deg2rad(double x) { return M_PI/180.0 * x; }

// Function slice
// This function slices into an iteratable
// Inputs:  iteratable to be slice
//          starting index (included in the slicing)
//          ending index (not included in the slicing)
// Returns: a neatly sliced iteratable
template <class T> T slice(T iteratable, int xstart, int xend) {
  // Declare subvariable
  T subiterate = new T[xend-xstart];

  for(int i=xstart; i< xend; i++) {

    subiterate[i-xstart] = iteratable[i];
  }
}

namespace ublox_ros
{

UBLOX_ROS::UBLOX_ROS() :
    nh_(), nh_private_("~")
{

    // Connect ROS topics
    pvt_pub_ = nh_.advertise<ublox::PositionVelocityTime>("PosVelTime", 10);
    relpos_pub_ = nh_.advertise<ublox::RelPos>("RelPos", 10);
    ecef_pub_ = nh_.advertise<ublox::PosVelEcef>("PosVelEcef", 10);
    survey_status_pub_ = nh_.advertise<ublox::SurveyStatus>("SurveyStatus", 10);
    eph_pub_ = nh_.advertise<ublox::Ephemeris>("Ephemeris", 10);
    geph_pub_ = nh_.advertise<ublox::GlonassEphemeris>("GlonassEphemeris", 10);
    obs_pub_ = nh_.advertise<ublox::ObsVec>("Obs", 10);
    // nav_sat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("NavSatFix");
    // nav_sat_status_pub_ = nh_.advertise<sensor_msgs::NavSatStatus>("NavSatStatus");

    // Connect ROS services
    cfg_val_get = nh_.advertiseService("CfgValGet", &UBLOX_ROS::cfgValGet, this);

    //Get the serial port
    std::string serial_port = nh_private_.param<std::string>("serial_port", "/dev/ttyACM0");

    // Get the log file
    std::string log_filename = nh_private_.param<std::string>("log_filename", "");

    std::cerr<<"Creating ublox parser\n";
    // create the parser
    ublox_ = new ublox::UBLOX(serial_port);
    std::cerr<<"Created ublox parser\n";

    //Get the number of rovers
    int rover_quantity = nh_private_.param<int>("rover_quantity", 0);

    //Get chain_level.
    //0 is stationary base
    // 1 to n-1 is
    int chain_level = nh_private_.param<int>("chain_level", 0x00);

    // set up RTK
    // Base (n local_host n local_port, n rover_host, n rover_port)
    if (chain_level == ublox::UBLOX::BASE){
        std::cerr<<"Initializing Base\n";

        base_host=NULL;
        base_port=NULL;

        //Initialize local arrays to contain parameters from xml file
        if(rover_quantity>0)
        {
            local_host = new std::string[rover_quantity];
            local_port = new uint16_t[rover_quantity];

            //Initialize rover arrays to contain parameters from xml file
            rover_host = new std::string[rover_quantity];
            rover_port = new uint16_t[rover_quantity];
        }
        else
        {
            local_host = NULL;
            local_port = NULL;
            rover_host = NULL;
            rover_port = NULL;
        }
        // Get Constallation settings
        uint32_t constellation [6];
        int gps = nh_private_.param<int>("GPS", 1); //GPS
        int glonas = nh_private_.param<int>("GLONAS", 0); //GLONAS
        int beidou = nh_private_.param<int>("BEIDOU", 0); //BEIDOU
        int galileo = nh_private_.param<int>("GALILEO", 1); //GALILEO
        int surveytime = nh_private_.param<int>("Surveytime", 120); //Stationary base survey time
        int surveyacc = nh_private_.param<int>("Surveyacc", 500000);  //Stationary base accuracy

        //Account for the case when no numbers are used for the first rover.
        int j = 0;
        if(nh_private_.hasParam("local_host")) {
            //The first local host corresponds to the first rover.
            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            //First rover.
            rover_host[0] = nh_private_.param<std::string>("rover_host", "localhost");
            rover_port[0] = nh_private_.param<int>("rover_port", 16145);

            //Let the program know that we have inputted the first rover.
            j=1;
        }

        //Input parameters from xml file into respective arrays for rovers.
        for(int i=1+j; i <= rover_quantity; i++) {
            local_host[i-1] = nh_private_.param<std::string>("local_host"+std::to_string(i), "localhost");
            local_port[i-1] = nh_private_.param<int>("local_port"+std::to_string(i), 16140);
            rover_host[i-1] = nh_private_.param<std::string>("rover_host"+std::to_string(i), "localhost");
            rover_port[i-1] = nh_private_.param<int>("rover_port"+std::to_string(i), 16145);
        }

        //Output the chain level we are at.
        std::cerr<<"Chain Level: " << chain_level << "\n";

        //Return outputs for all local hosts and rovers hooked up.
        for(int i = 0; i < rover_quantity; i++) {
            std::cerr<<"local_host " + std::to_string(i+1) + ": " << local_host[i] << "\n";
            std::cerr<<"local_port " + std::to_string(i+1) + ": " << local_port[i] << "\n";
            std::cerr<<"rover_host " + std::to_string(i+1) + ": " << rover_host[i] << "\n";
            std::cerr<<"rover_port " + std::to_string(i+1) + ": " << rover_port[i] << "\n";
        }

        //Determine whether the base is moving or stationary
        std::string base_type = nh_private_.param<std::string>("base_type", "stationary");

        std::cerr<<"About to init base\n";
        ublox_->initBase(local_host, local_port, rover_host, rover_port,
          base_type, rover_quantity, gps, glonas, beidou, galileo, surveytime,
          surveyacc);
    }
    // Rover(1 local_host 1 local_port 1 base_host 1 base_port)
    else if (rover_quantity == 0){

        std::cerr<<"Initializing Rover\n";

        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[1];
        uint16_t* local_port = new uint16_t[1];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        rover_host = NULL;
        rover_port = NULL;

        // Get Constallation settings
        uint32_t constellation [4];
        constellation[0] = nh_private_.param<int>("GPS", 1);
        constellation[1] = nh_private_.param<int>("GLONAS", 0);
        constellation[2] = nh_private_.param<int>("BEIDOU", 0);
        constellation[3] = nh_private_.param<int>("GALILEO", 1);

        if(nh_private_.hasParam("local_host")) {
            std::string test = nh_private_.param<std::string>("local_host", "localhost");
            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            base_host[0] = nh_private_.param<std::string>("base_host", "localhost");
            base_port[0] = nh_private_.param<int>("base_port", 16145);
        }
        else {
          local_host[0] = nh_private_.param<std::string>("local_host1", "localhost");
          local_port[0] = nh_private_.param<int>("local_port1", 16140);
          base_host[0] = nh_private_.param<std::string>("base_host1", "localhost");
          base_port[0] = nh_private_.param<int>("base_port1", 16145);
        }
        std::cerr<<"Chain Level: " << chain_level << "\n";
        std::cerr<<"Local Host: "<<local_host[0]<<"\n";
        std::cerr<<"Local Port: "<<local_port[0]<<"\n";
        std::cerr<<"Base Host: "<<base_host[0]<<"\n";
        std::cerr<<"Base Port: "<<base_port[0]<<"\n";

        ublox_->initRover(local_host[0], local_port[0], base_host[0], base_port[0]);
    }
    // Brover(1 base_host 1 base_port n local_host n local_port n rover_host n rover_port)
    else if (rover_quantity>=0) {
        std::cerr<<"Initializing Brover\n";

        // Initialize local arrays to contain parameters from xml file
        // The first local_host and local_port correspond to the base.
        std::string* local_host = new std::string[rover_quantity+1];
        uint16_t* local_port = new uint16_t[rover_quantity+1];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[rover_quantity];
        uint16_t* rover_port = new uint16_t[rover_quantity];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        // Fill base arrays with their single values
        base_host[0] = nh_private_.param<std::string>("base_host", "localhost");
        base_port[0] = nh_private_.param<int>("base_port", 16140);

        // Get Constallation settings
        uint32_t constellation [6];
        int gps = nh_private_.param<int>("GPS", 1);
        int glonas = nh_private_.param<int>("GLONAS", 0);
        int beidou = nh_private_.param<int>("BEIDOU", 0);
        int galileo = nh_private_.param<int>("GALILEO", 1);

        int j = 0;
        if(nh_private_.hasParam("local_host")) {

            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            rover_host[0] = nh_private_.param<std::string>("rover_host", "localhost");
            rover_port[0] = nh_private_.param<int>("rover_port", 16145);
            j=1;
        }

        //Input parameters from xml file into respective arrays
        for(int i=1+j; i <= rover_quantity; i++) {
            local_host[i-1] = nh_private_.param<std::string>("local_host"+std::to_string(i), "localhost");
            local_port[i-1] = nh_private_.param<int>("local_port"+std::to_string(i), 16140);
            rover_host[i-1] = nh_private_.param<std::string>("rover_host"+std::to_string(i), "localhost");
            rover_port[i-1] = nh_private_.param<int>("rover_port"+std::to_string(i), 16145);
            j = i;
        }

        // Add in extra local host values.
        local_host[j] = nh_private_.param<std::string>("local_host"+std::to_string(j+1), "localhost");
        local_port[j] = nh_private_.param<int>("local_port"+std::to_string(j+1), 16140);

        // Output chain level
        std::cerr<<"Chain Level: " << chain_level << "\n";

        // Output base host
        std::cerr<<"Base Host: "<<base_host[0]<<"\n";

        // Output base port
        std::cerr<<"Base Port: "<<base_port[0]<<"\n";
        for(int i = 0; i < rover_quantity; i++) {
            std::cerr<<"local_host " + std::to_string(i+1) + ": " << local_host[i] << "\n";
            std::cerr<<"local_port " + std::to_string(i+1) + ": " << local_port[i] << "\n";
            std::cerr<<"rover_host " + std::to_string(i+1) + ": " << rover_host[i] << "\n";
            std::cerr<<"rover_port " + std::to_string(i+1) + ": " << rover_port[i] << "\n";
        }


        //Determine whether the base is moving or stationary
        std::string base_type = "moving";

        // Initiate the Brover
        ublox_->initBrover(local_host, local_port, base_host, base_port,
           rover_host, rover_port, base_type, rover_quantity, gps,
            glonas, beidou, galileo);

    }

    // Check if there is a arrow
    if (nh_private_.hasParam("arrowbase") && nh_private_.hasParam("arrowtip")) {

      // If there is an arrow , then we need to subscribe to the base
      std::string arrowbase = nh_private_.param<std::string>("arrowbase", "/brover");
      // and tip of the arrow for /RelPos
      std::string arrowtip = nh_private_.param<std::string>("arrowtip", "/rover");

      // Call the first subscriber
      sub1 = nh_.subscribe(arrowbase+"/RelPos", 10, &UBLOX_ROS::cb_rov1, this);

      // Call the second subscriber
      sub2 = nh_.subscribe(arrowtip+"/RelPos", 10, &UBLOX_ROS::cb_rov2, this);

      // Make the arrow flag true. This flag is used in the relposCB function in
      // in order to determine if the vector math function needs to be called or
      // not.
      arrow_flag = true;
    }

    // connect callbacks
    createCallback(ublox::CLASS_NAV, ublox::NAV_PVT, pvtCB, NAV_PVT);
    createCallback(ublox::CLASS_NAV, ublox::NAV_RELPOSNED, relposCB, NAV_RELPOSNED);
    createCallback(ublox::CLASS_NAV, ublox::NAV_POSECEF, posECEFCB, NAV_POSECEF);
    createCallback(ublox::CLASS_NAV, ublox::NAV_VELECEF, velECEFCB, NAV_VELECEF);
    createCallback(ublox::CLASS_NAV, ublox::NAV_SVIN, svinCB, NAV_SVIN);
    createCallback(ublox::CLASS_RXM, ublox::RXM_RAWX, obsCB, RXM_RAWX);

    if (!log_filename.empty())
    {
        ublox_->initLogFile(log_filename);
        //ublox_->readFile(log_filename);

    }
}

UBLOX_ROS::~UBLOX_ROS()
{
    if (ublox_)
        delete ublox_;
    if (local_host!=NULL)
        delete local_host;
    if (local_port!=NULL)
        delete local_port;
    if (rover_host!=NULL)
        delete rover_host;
    if (rover_port!=NULL)
        delete rover_port;
    if (base_host!=NULL)
        delete base_host;
    if (base_port!=NULL)
        delete base_port;
    
}

// Callback function for subscriber to RelPos for a given RelPos message.
// NOTE: This message is not the same as ublox::NAV_RELPOSNED_t, since that one
// deals with messages from the f9p
void UBLOX_ROS::cb_rov1(const ublox::RelPos &msg) {
    ned_1[0] = msg.relPosNED[0];  //North
    ned_1[1] = msg.relPosNED[1];  //East
    ned_1[2] = msg.relPosNED[2];  //Down
}

// Callback function for subscriber to second RelPos.
// NOTE: This message is not the same as ublox::NAV_RELPOSNED_t, since that one
// deals with messages from the f9p
void UBLOX_ROS::cb_rov2(const ublox::RelPos &msg) {
    ned_2[0] = msg.relPosNED[0];  //North
    ned_2[1] = msg.relPosNED[1];  //East
    ned_2[2] = msg.relPosNED[2];  //Down
}

void UBLOX_ROS::pvtCB(const ublox::NAV_PVT_t& msg)
{
    pos_tow_ = msg.iTOW;
    ublox::PositionVelocityTime out;
    // out.iTOW = msg.iTow;
    out.header.stamp = ros::Time::now(); ///TODO: Do this right
    out.year = msg.year;
    out.month = msg.month;
    out.day = msg.day;
    out.hour = msg.hour;
    out.min = msg.min;
    out.sec = msg.sec;
    out.nano = msg.nano;
    out.tAcc = msg.tAcc;
    out.valid = msg.valid;
    out.fixType = msg.fixType;
    out.flags = msg.flags;
    out.flags2 = msg.flags2;
    out.numSV = msg.numSV;
    out.lla[0] = msg.lat*1e-7;
    out.lla[1] = msg.lon*1e-7;
    out.lla[2] = msg.height*1e-3;
    out.hMSL = msg.hMSL*1e-3;
    out.hAcc = msg.hAcc*1e-3;
    out.vAcc = msg.vAcc*1e-3;
    out.velNED[0] = msg.velN*1e-3;
    out.velNED[1] = msg.velE*1e-3;
    out.velNED[2] = msg.velD*1e-3;
    out.gSpeed = msg.gSpeed*1e-3;
    out.headMot = msg.headMot*1e-5;
    out.sAcc = msg.sAcc*1e-3;
    out.headAcc = msg.headAcc*1e-5;
    out.pDOP = msg.pDOP*0.01;
    out.headVeh = msg.headVeh*1e-5;
    pvt_pub_.publish(out);

    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.fix = out.fixType;
    ecef_msg_.lla[0] = out.lla[0];
    ecef_msg_.lla[1] = out.lla[1];
    ecef_msg_.lla[2] = out.lla[2];
    ecef_msg_.horizontal_accuracy = out.hAcc;
    ecef_msg_.vertical_accuracy = out.vAcc;
    ecef_msg_.speed_accuracy = out.sAcc;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        ecef_pub_.publish(ecef_msg_);
}


void UBLOX_ROS::relposCB(const ublox::NAV_RELPOSNED_t& msg)
{
    // Create the message to be outputted
    ublox::RelPos out;


    // out.iTOW = msg.iTow*1e-3;
    out.header.stamp = ros::Time::now(); /// TODO: do this right
    out.refStationId = msg.refStationId;
    out.relPosNED[0] = msg.relPosN*1e-2;
    out.relPosNED[1] = msg.relPosE*1e-2;
    out.relPosNED[2] = msg.relPosD*1e-2;
    out.relPosLength = msg.relPosLength*1e-2;
    out.relPosHeading = deg2rad(msg.relPosHeading*1e-5);
    out.relPosHPNED[0] = msg.relPosHPN*1e-3*.1;
    out.relPosHPNED[1] = msg.relPosHPE*1e-3*.1;
    out.relPosHPNED[2] = msg.relPosHPD*1e-3*.1;
    out.relPosHPLength = msg.relPosHPLength*1e-3*.1;
    out.accNED[0] = msg.accN*1e-3*.1;
    out.accNED[1] = msg.accE*1e-3*.1;
    out.accNED[2] = msg.accD*1e-3*.1;
    out.accLength = msg.accLength*1e-3*.1;
    out.accHeading = deg2rad(msg.accHeading*1e-5);
    out.flags = msg.flags;

    if (arrow_flag == true) {

    // Perform vector_math and assign values to arrow. (see ublox_ros.h for
    // variable declarations)
    ublox_->vector_math(ned_1, ned_2, arrow);

    // Assign all the values
    out.arrowNED[0] = arrow[0];
    out.arrowNED[1] = arrow[1];
    out.arrowNED[2] = arrow[2];
    out.arrowLength = arrow[3];
    out.arrowRPY[0] = arrow[4];
    out.arrowRPY[1] = arrow[5];
    out.arrowRPY[2] = arrow[6];
  }
    // Publish the RelPos ROS message
    relpos_pub_.publish(out);
}

void UBLOX_ROS::svinCB(const ublox::NAV_SVIN_t& msg)
{
    ublox::SurveyStatus out;
    out.header.stamp = ros::Time::now(); /// TODO: do this right
    out.dur = msg.dur;
    out.meanXYZ[0] = msg.meanX*1e-2;
    out.meanXYZ[1] = msg.meanY*1e-2;
    out.meanXYZ[2] = msg.meanZ*1e-2;
    out.meanXYZHP[0] = msg.meanXHP*1e-3;
    out.meanXYZHP[1] = msg.meanYHP*1e-3;
    out.meanXYZHP[2] = msg.meanZHP*1e-3;
    out.meanAcc = msg.meanAcc;
    out.obs = msg.obs;
    out.valid = msg.valid;
    out.active = msg.active;
    survey_status_pub_.publish(out);

}

void UBLOX_ROS::posECEFCB(const ublox::NAV_POSECEF_t& msg)
{
    pos_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.position[0] = msg.ecefX*1e-2;
    ecef_msg_.position[1] = msg.ecefY*1e-2;
    ecef_msg_.position[2] = msg.ecefZ*1e-2;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        ecef_pub_.publish(ecef_msg_);
    ecef_pub_.publish(ecef_msg_);

}

void UBLOX_ROS::velECEFCB(const ublox::NAV_VELECEF_t& msg)
{
    vel_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.velocity[0] = msg.ecefVX*1e-2;
    ecef_msg_.velocity[0] = msg.ecefVY*1e-2;
    ecef_msg_.velocity[0] = msg.ecefVZ*1e-2;

    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        ecef_pub_.publish(ecef_msg_);
    ecef_pub_.publish(ecef_msg_);
}

void UBLOX_ROS::obsCB(const ublox::RXM_RAWX_t &msg)
{
    ublox::ObsVec out;
    UTCTime utc =UTCTime::fromGPS(msg.week, msg.rcvTow*1e3);
    out.header.stamp.sec = utc.sec;
    out.header.stamp.nsec = utc.nsec;
    for (int i = 0; i < msg.numMeas; i++)
    {
        out.obs[i].sat = msg.meas[i].svId;
        out.obs[i].gnssID = msg.meas[i].gnssId;
        out.obs[i].signal = ublox::sigId(msg.meas[i].gnssId, msg.meas[i].sigId);
        switch (out.obs[i].signal)
        {
        case ublox::Observation::GPS_L1_CA:
        case ublox::Observation::GALILEO_E1_B:
        case ublox::Observation::GALILEO_E1_C:
        case ublox::Observation::QZSS_L1_CA:
            out.obs[i].freq = Ephemeris::GPS_FREQL1;
            break;
        case ublox::Observation::GPS_L2_CL:
        case ublox::Observation::GPS_L2_CM:
            out.obs[i].freq = Ephemeris::GPS_FREQL2;
            break;
        case ublox::Observation::GLONASS_L1:
            out.obs[i].freq = GlonassEphemeris::FREQ1_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ1_GLO;
            break;
        case ublox::Observation::GLONASS_L2:
            out.obs[i].freq = GlonassEphemeris::FREQ2_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ2_GLO;
            break;
            // These may not be right
//        case ublox::Observation::GALILEO_E5_BI:
//        case ublox::Observation::GALILEO_E5_BQ:
//            out.obs[i].freq = Ephemeris::GALILEO_FREQL5b;
//            break;
//        case ublox::Observation::BEIDOU_B1I_D1:
//        case ublox::Observation::BEIDOU_B1I_D2:
//            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B1;
//            break;
//        case ublox::Observation::BEIDOU_B2I_D1:
//        case ublox::Observation::BEIDOU_B2I_D2:
//            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B2;
//            break;
        default:      // and tip of the arrow for /RelPos
            out.obs[i].freq = 0;
            break;
        }
        out.obs[i].cno = msg.meas[i].cno;
        out.obs[i].locktime = msg.meas[i].locktime;
        out.obs[i].P = msg.meas[i].prMeas;
        out.obs[i].L = msg.meas[i].cpMeas;
        out.obs[i].D = msg.meas[i].doMeas;
        out.obs[i].stdevP = 0.01 * pow(2, msg.meas[i].prStdev);
        out.obs[i].stdevL = 0.004 * msg.meas[i].cpStdev;
        out.obs[i].stdevD = 0.002 * pow(2, msg.meas[i].doStdev);

        // indicate cycle slip
        if (msg.meas[i].cpMeas != 0.0
            && (msg.meas[i].trkStat & ublox::RXM_RAWX_t::trkStat_HalfCyc | ublox::RXM_RAWX_t::trkStat_subHalfCyc))
        {
            out.obs[i].LLI =  ublox::Observation::LLI_HALF_CYCLE_AMB;
        }
        else
        {
            out.obs[i].LLI = 0;
        }
    }
    obs_pub_.publish(out);
}

void UBLOX_ROS::ephCB(const Ephemeris &eph)
{
    ublox::Ephemeris out;
    out.header.stamp = ros::Time::now();

    out.sat = eph.sat;
    out.gnssID = eph.gnssID;
    out.toe.sec = eph.toe.sec;
    out.toe.nsec = eph.toe.nsec;
    out.toc.sec = eph.toc.sec;
    std::cerr<<"About to spin\n";
    out.toc.nsec = eph.toc.nsec;

    out.tow = eph.tow;
    out.iodc = eph.iodc;
    out.iode = eph.iode;
    out.week = eph.week;
    out.toes = eph.toes;
    out.tocs = eph.tocs;
    out.health = eph.health;
    out.alert_flag = eph.alert_flag;
    out.anti_spoof = eph.anti_spoof;
    out.code_on_L2 = eph.code_on_L2;
    out.ura = eph.ura;
    out.L2_P_data_flag = eph.L2_P_data_flag;
    out.fit_interval_flag = eph.fit_interval_flag;
    out.age_of_data_offset = eph.age_of_data_offset;
    out.tgd[0] = eph.tgd[0];
    out.tgd[1] = eph.tgd[1];
    out.tgd[2] = eph.tgd[2];
    out.tgd[3] = eph.tgd[3];
    out.af2 = eph.af2;
    out.af1 = eph.af1;
    out.af0 = eph.af0;
    out.m0 = eph.m0;
    out.delta_n = eph.delta_n;
    out.ecc = eph.ecc;
    out.sqrta = eph.sqrta;
    out.omega0 = eph.omega0;
    out.i0 = eph.i0;
    out.w = eph.w;
    out.omegadot = eph.omegadot;
    out.idot = eph.idot;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.cic = eph.cic;
    out.cis = eph.cis;

    eph_pub_.publish(out);
}

void UBLOX_ROS::gephCB(const GlonassEphemeris &eph)
{
    ublox::GlonassEphemeris out;
    out.header.stamp = ros::Time::now();

    out.sat = eph.sat;
    out.gnssID = eph.gnssID;

    out.toe.sec = eph.toe.sec;
    out.toe.nsec = eph.toe.nsec;
    out.tof.sec = eph.tof.sec;
    out.tof.nsec = eph.tof.nsec;

    out.iode = eph.iode;
    out.frq = eph.frq;
    out.svh = eph.svh;
    out.sva = eph.sva;
    out.age = eph.age;
    out.pos[0] = eph.pos[0];
    out.pos[1] = eph.pos[1];
    out.pos[2] = eph.pos[2];
    out.vel[0] = eph.vel[0];
    out.vel[1] = eph.vel[1];
    out.vel[2] = eph.vel[2];
    out.acc[0] = eph.acc[0];
    out.acc[1] = eph.acc[1];
    out.acc[2] = eph.acc[2];
    out.taun = eph.taun;
    out.gamn = eph.gamn;
    out.dtaun = eph.dtaun;

    geph_pub_.publish(out);
}

bool UBLOX_ROS::cfgValGet(ublox::CfgValGet::Request &req, ublox::CfgValGet::Response &res)
{
    ublox::CFG_VALGET_t request;
    request.version=req.version;
    request.layer=req.layer;
    request.position=req.position;
    request.cfgDataKey=req.keys;

    ublox::CFG_VALGET_t response = ublox_->cfgValGet(request);
    
    res.version=response.version;
    res.layer=response.layer;
    res.position=response.position;
    res.cfgData.push_back(response.cfgData);
    return true;
}

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_ros");

    ublox_ros::UBLOX_ROS Thing;

    ros::spin();
}
