#pragma once
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>
#include <atomic>
#include <functional>
#include <vector> 
#include <iomanip>
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <netinet/in.h>
#include </home/dana/mavlink/build/include/mavlink/common/mavlink.h>
#include<chrono>
#include <mutex>
#include <iomanip>
#include <string>
#include <nlohmann/json.hpp>
#include <fstream>
#include "../geodesic/getCoordinate.h"
using namespace std;
using json = nlohmann::json;



struct  MavData 
{
    struct global_position 
    {
        uint32_t time_boot_ms; //ms, Временная метка (время с момента загрузки системы) 
        int32_t lat;//degE7, Широта
        int32_t lon;//degE7, Долгота
        int32_t alt;//mm, Высота над уровнем моря (MSL)
        int32_t relative_alt;//mm, Высота над землей
        int16_t vx;//cm/s, Скорость движения по земле X
        int16_t vy;//cm/s, Скорость движения по земле Y
        int16_t vz;//cm/s, Скорость движения по земле Z
        int16_t hdg;//cdeg, Направление движения транспортного средства (угол рыскания)
    } position;


    struct attitude
    {
        uint32_t time_boot_ms;//ms	Timestamp (time since system boot).
        float roll;//rad	Roll angle (-pi..+pi)
        float pitch;//	rad	Pitch angle (-pi..+pi)
        float yaw;//rad	Yaw angle (-pi..+pi)
        float rollspeed;//	rad/s	Roll angular speed
        float pitchspeed;//rad/s	Pitch angular speed
        float yawspeed;//rad/s	Yaw angular speed
    } att;
    struct altitude
    {
        uint64_t time_usec;//us	Timestamp (UNIX Epoch time or time since system boot). 
        float altitude_monotonic;//This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). 
        float altitude_amsl;//	m	This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). 
        float altitude_local;//m	This is the local altitude in the local coordinate frame.
        float altitude_relative;//m	This is the altitude above the home position. 
        float altitude_terrain;//m	This is the altitude above terrain.
        float bottom_clearance;//m	This is not the altitude, but the clear space below the system according to the fused clearance estimate. 


    } alt;
    struct battery_status
    {
        uint8_t id;//Battery ID
        uint8_t battery_function;//	MAV_BATTERY_FUNCTION	Function of the battery
        uint8_t type;//MAV_BATTERY_TYPE	Type (chemistry) of the battery
        int16_t temperature;//degC		Temperature of the battery. 
        vector <uint16_t> voltages;//mV		Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). 
        int16_t current_battery;//	cA		Battery current, -1: autopilot does not measure the current
        int32_t current_consumed;//mAh		Consumed charge, -1: autopilot does not provide consumption estimate
        int32_t energy_consumed;//hJ		Consumed energy, -1: autopilot does not provide energy consumption estimate
        uint8_t  battery_remaining;//%		Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
        int32_t  time_remaining;//s		Remaining battery time, 0: autopilot does not provide remaining battery time estimate
        uint8_t  charge_state;// 	MAV_BATTERY_CHARGE_STATE	State for extent of discharge, provided by autopilot for warning or external reactions
        vector <uint16_t> voltages_ext;//mV		Battery voltages for cells 11 to 14. 
        uint8_t  mode;//MAV_BATTERY_MODE	Battery mode. Default (0) is that battery mode reporting is not supported or battery is in normal-use mode.
        uint32_t  fault_bitmask;//MAV_BATTERY_FAULT	Fault/health indications. 

    } battery;

    struct GPS_RAW_INT
    {
        uint64_t time_usec;//us		Timestamp (UNIX Epoch time or time since system boot).
        uint8_t fix_type;//GPS_FIX_TYPE	GPS fix type.
        int32_t lat;//degE7		Latitude (WGS84, EGM96 ellipsoid)
        int32_t lon;//degE7		Longitude
        int32_t alt;//mm		Altitude (MSL).
        uint16_t eph;//GPS HDOP horizontal dilution of position (unitless * 100).
        uint16_t epv;//GPS VDOP vertical dilution of position (unitless * 100). 
        uint16_t vel;//cm/s		GPS ground speed. 
        uint16_t cog;//cdeg		Course over ground (NOT heading, but direction of movement) in degrees *
        uint8_t satellites_visible;//Number of satellites visible. 
        int32_t alt_ellipsoid;//mm		Altitude (above WGS84, EGM96 ellipsoid).
        uint32_t h_acc;//mm		Position uncertainty.
        uint32_t v_acc;//mm		Altitude uncertainty.
        uint32_t vel_acc;//mm		Speed uncertainty.
        uint32_t hdg_acc;//degE5		Heading / track uncertainty
        uint16_t yaw;//cdeg		Yaw in earth frame from north. 


    } gps;

        struct LOCAL_POSITION_NED
    {
        uint32_t time_boot_ms;//ms	Timestamp (time since system boot).
        float x;//m	X Position
        float y;//	m	Y Position
        float z;//m	Z Position
        float vx;//	m/s	X Speed
        float vy;//	m/s	Y Speed
        float vz;//m/s	Z Speed
        

    } local_pos;

        struct VFR_HUD
    {   
       float  airspeed;//m/s	Vehicle speed in form appropriate for vehicle type. 
       float  groundspeed;//m/s	Current ground speed.
       int16_t  heading;//deg	Current heading in compass units (0-360, 0=north).
       uint16_t  throttle;//%	Current throttle setting (0 to 100).
       float  alt;//m	Current altitude (MSL).
       float  climb;//	m/s	Current climb rate.


    } vfr;

        struct SERVO_OUTPUT_RAW
    {
        uint64_t time_usec;//us	Timestamp (since system boot).
        uint32_t active;//Active outputs
        vector <float>actuator;//Servo / motor output array values.

    } raw;

            struct EXTENDED_SYS_STATE
    {
       uint8_t vtol_state;//MAV_VTOL_STATE	The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
       uint8_t landed_state;//MAV_LANDED_STATE	The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.

    } ext_sys;

        struct HOME_POSITION
    {
        int32_t latitude;//degE7	Latitude (WGS84)
        int32_t longitude;//degE7	Longitude (WGS84)
        int32_t altitude;//mm	Altitude (MSL). Positive for up.
        float x;//	m	Local X position of this position in the local coordinate frame (NED)
        float y;//m	Local Y position of this position in the local coordinate frame (NED)
        float z;//m	Local Z position of this position in the local coordinate frame (NED: positive "down")
        vector<float> q;//Quaternion indicating world-to-surface-normal and heading transformation of the takeoff position. 
        float approach_x;//m	Local X position of the end of the approach vector.
        float approach_y;//m	Local Y position of the end of the approach vector. 
        float approach_z;//m	Local Z position of the end of the approach vector. 
        uint64_t time_usec;//us	Timestamp (UNIX Epoch time or time since system boot). 


    } home_pos;

        struct HEARTBEAT
    {
        uint8_t type;//	MAV_TYPE	Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.).
        uint8_t autopilot;//	MAV_AUTOPILOT	Autopilot type / class. 
        uint8_t base_mode;//MAV_MODE_FLAG	System mode bitmap.
        uint32_t custom_mode;//A bitfield for use for autopilot-specific flags
        uint8_t system_status;//	MAV_STATE	System status flag.

        

    } heartbeat;


            struct SYS_STATUS
    {
        uint32_t onboard_control_sensors_present;//
        uint32_t onboard_control_sensors_enabled;//
        uint32_t onboard_control_sensors_health;//
        uint16_t load;//d%		Maximum usage in percent of the mainloop time.
        uint16_t voltage_battery;//mV		Battery voltage,
        int16_t current_battery;//cA		Battery current,
        int8_t battery_remaining;//%		Battery energy
        uint16_t drop_rate_comm;//c%		Communication drop rate,
        uint16_t errors_comm;//
        uint16_t errors_count1;//
        uint16_t errors_count2;//
        uint16_t errors_count3;//
        uint16_t errors_count4;//
        uint32_t onboard_control_sensors_present_extended;//
        uint32_t onboard_control_sensors_enabled_extended;//
        uint32_t onboard_control_sensors_health_extended;//

    } sys;

        struct SCALED_IMU
     
    {
        uint32_t time_boot_ms;//ms	Timestamp (time since system boot).
        int16_t xacc;//mG	X acceleration
        int16_t yacc;//mG	X acceleration
        int16_t zacc;//	mG	Z acceleration
        int16_t xgyro;//mrad/s	Angular speed around X axis
        int16_t ygyro;//mrad/s	Angular speed around Y axis
        int16_t zgyro;//mrad/s	Angular speed around Z axis
        int16_t xmag;//	mgauss	X Magnetic field
        int16_t ymag;//mgauss	Y Magnetic field
        int16_t zmag;//mgauss	Z Magnetic field
        int16_t temperature;//cdegC	Temperature, 0

    } imu;

        struct ODOMETRY
    {
        uint64_t time_usec;//us		Timestamp (UNIX Epoch time or time since system boot). 
        uint8_t frame_id;//MAV_FRAME	Coordinate frame of reference for the pose data.
        uint8_t child_frame_id;//MAV_FRAME	Coordinate frame of reference for the velocity in free space (twist) data.
        float x;//m		X Position
        float y;//m		Y Position
        float z;//m		Z Position
        vector<float> q;//Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        float vx;//m/s		X linear speed
        float vy;//m/s		Y linear speed
        float vz;//m/s		Z linear speed
        float rollspeed;//rad/s		Roll angular speed
        float pitchspeed;//rad/s		Pitch angular speed
        float yawspeed;//rad/s		Yaw angular speed
        vector<float>	 pose_covariance;//Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw;
        vector<float>	 velocity_covariance;//Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; 
        uint8_t	 reset_counter;//Estimate reset counter. 
        uint8_t	 estimator_type;//Type of estimator that is providing the odometry.
        uint8_t	 quality;//Optional odometry quality metric as a percentage.

    } odometry;



      struct STATUS_TEXT
    {
       uint8_t severity;
       vector<char> text;
       uint16_t id;
       uint8_t chunk_seq;

    } stat_text;
};



class MavlinkReceiver 
{
private:
    getCoordinate coordinate;
    int socket_fd;
    MavData MsgData;
    struct sockaddr_in addr = {};
    static constexpr int BUFFER_LENGTH = 2041;
    mutex mtx;
    int sysid=0;
    int compid=0;
    json j;
    bool fl = 0;
    struct sockaddr_in src_addr = {};
     socklen_t src_addr_len;
    std::thread thread_;
    bool running_ = false;
    bool src_addr_set;
public:
MavlinkReceiver() : socket_fd(-1) {}
    ~MavlinkReceiver() {disconnect();}


//main_methods
int connect();

void disconnect();

void ProcessData();

void send_heartbeat();

//utils
void printBuffer(const uint8_t* buffer, uint16_t length);

int send_message(mavlink_message_t* msg);


//commands
int takeoff(float h);

int land();

int return_to_lunch();

int change_speed(float speed);


//mission_commands
int parse_mission_json(string s);

int upload_mission();

int download_mission();

int set_mission(int a);

int start_mission();

int pause_mission();

int continue_mission();

int clear_mission();



//offboard_commands
int set_offboard();


int send_offboard();
};


