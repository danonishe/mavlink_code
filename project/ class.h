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
#include </home/dana/mavlink/build/include/mavlink/common/mavlink.h>


using namespace std;

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
    int udp_socket_;
    struct sockaddr_in sockaddr_;
    static constexpr int BUFFER_LENGTH = 2041;
    MavData MsgData;

    std::thread thread_;
    bool running_ = false;



public:
    MavlinkReceiver() : udp_socket_(-1) {}

    ~MavlinkReceiver() 
    {
        disconnect();
    }

    bool connect(int port) 
    {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) 
        {
            std::cerr << "Ошибка создания сокета\n";
            return false;
        }

        memset(&sockaddr_, 0, sizeof(sockaddr_));
        sockaddr_.sin_family = AF_INET;
        sockaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
        sockaddr_.sin_port = htons(port);

        if (bind(udp_socket_, (struct sockaddr *) &sockaddr_, sizeof(sockaddr_)) < 0)
         {
            std::cerr << "Ошибка привязки сокета к порту\n";
            close(udp_socket_);
            udp_socket_ = -1;
            return false;
        }
         running_ = true;
 
        thread_ = std::thread(&MavlinkReceiver::ProcessData, this);
    
        return true;
    }

    void disconnect() 
    {
        if (udp_socket_ >= 0) 
        {
            close(udp_socket_);
            udp_socket_ = -1;
        }

                running_ = false;
    
        if (thread_.joinable()) {
            thread_.join();
        }
        
    }

        
    void ProcessData()
     {
       while (running_){

            if (udp_socket_ < 0) 
        {
            std::cerr << "Сокет не подключен\n";
            return;
        }

        uint8_t buffer[BUFFER_LENGTH];
        ssize_t num_bytes = recv(udp_socket_, buffer, sizeof(buffer), 0);
        if (num_bytes <= 0) 
        {
            std::cerr << "Ошибка приема данных\n";
            return;
        }

        mavlink_message_t msg;
        mavlink_status_t status;
        for (ssize_t i = 0; i < num_bytes; ++i) 
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) 
            {
                   
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) 
                    {
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);
                        MsgData.position.time_boot_ms = pos.time_boot_ms;
                        MsgData.position.lat = pos.lat;
                        MsgData.position.lon = pos.lon;
                        MsgData.position.alt = pos.alt;
                        MsgData.position.relative_alt = pos.relative_alt;
                        MsgData.position.vx = pos.vx;
                        MsgData.position.vy = pos.vy;
                        MsgData.position.vz = pos.vz;
                        MsgData.position.hdg = pos.hdg;
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) 
                    {
                       
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        MsgData.att.time_boot_ms = attitude.time_boot_ms;
                        MsgData.att.roll = attitude.roll;
                        MsgData.att.pitch = attitude.pitch;
                        MsgData.att.yaw = attitude.yaw;
                        MsgData.att.rollspeed = attitude.rollspeed;
                        MsgData.att.pitchspeed = attitude.pitchspeed;
                        MsgData.att.yawspeed = attitude.yawspeed;
                    }
                

                    if (msg.msgid == MAVLINK_MSG_ID_ALTITUDE) 
                    {
                         
                        mavlink_altitude_t altitude_msg;
                        mavlink_msg_altitude_decode(&msg, &altitude_msg);
                        MsgData.alt.time_usec = altitude_msg.time_usec;
                        MsgData.alt.altitude_monotonic = altitude_msg.altitude_monotonic;
                        MsgData.alt.altitude_amsl = altitude_msg.altitude_amsl;
                        MsgData.alt.altitude_local = altitude_msg.altitude_local;
                        MsgData.alt.altitude_relative = altitude_msg.altitude_relative;
                        MsgData.alt.altitude_terrain = altitude_msg.altitude_terrain;
                        MsgData.alt.bottom_clearance = altitude_msg.bottom_clearance;
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) 
                    {
                        
                        mavlink_battery_status_t battery_status_msg;
                        mavlink_msg_battery_status_decode(&msg, &battery_status_msg);
                        MsgData.battery.id = battery_status_msg.id;
                        MsgData.battery.battery_function = battery_status_msg.battery_function;
                        MsgData.battery.type = battery_status_msg.type;
                        MsgData.battery.temperature = battery_status_msg.temperature;
                        for (int i = 0; i < 10; ++i) {
                            MsgData.battery.voltages.push_back(battery_status_msg.voltages[i]);
                        }
                        MsgData.battery.current_battery = battery_status_msg.current_battery;
                        MsgData.battery.current_consumed = battery_status_msg.current_consumed;
                        MsgData.battery.energy_consumed = battery_status_msg.energy_consumed;
                        MsgData.battery.battery_remaining = battery_status_msg.battery_remaining;
                        MsgData.battery.time_remaining = battery_status_msg.time_remaining;
                        MsgData.battery.charge_state = battery_status_msg.charge_state;

                            for (int i = 0; i < 4; ++i) {
                            MsgData.battery.voltages_ext.push_back(battery_status_msg.voltages_ext[i]);
                        }
                
                        MsgData.battery.mode = battery_status_msg.mode;
                        MsgData.battery.fault_bitmask = battery_status_msg.fault_bitmask;
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) 
                    {
                     
                        mavlink_gps_raw_int_t gps_msg;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps_msg);
                        MsgData.gps.time_usec = gps_msg.time_usec;
                        MsgData.gps.fix_type = gps_msg.fix_type;
                        MsgData.gps.lat = gps_msg.lat;
                        MsgData.gps.lon = gps_msg.lon;
                        MsgData.gps.alt = gps_msg.alt;
                        MsgData.gps.eph = gps_msg.eph;
                        MsgData.gps.epv = gps_msg.epv;
                        MsgData.gps.vel = gps_msg.vel;
                        MsgData.gps.cog = gps_msg.cog;
                        MsgData.gps.satellites_visible = gps_msg.satellites_visible;
                        MsgData.gps.alt_ellipsoid = gps_msg.alt_ellipsoid;
                        MsgData.gps.h_acc = gps_msg.h_acc;
                        MsgData.gps.v_acc = gps_msg.v_acc;
                        MsgData.gps.vel_acc = gps_msg.vel_acc;
                        MsgData.gps.hdg_acc = gps_msg.hdg_acc;
                        MsgData.gps.yaw = gps_msg.yaw;
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) 
                    {
                          
                        mavlink_local_position_ned_t local_pos_msg;
                        mavlink_msg_local_position_ned_decode(&msg, &local_pos_msg);
                        MsgData.local_pos.time_boot_ms = local_pos_msg.time_boot_ms;
                        MsgData.local_pos.x = local_pos_msg.x;
                        MsgData.local_pos.y = local_pos_msg.y;
                        MsgData.local_pos.z = local_pos_msg.z;
                        MsgData.local_pos.vx = local_pos_msg.vx;
                        MsgData.local_pos.vy = local_pos_msg.vy;
                        MsgData.local_pos.vz = local_pos_msg.vz;
                    }

              
                    if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) 
                    {
                         
                        mavlink_vfr_hud_t vfr_msg;
                        mavlink_msg_vfr_hud_decode(&msg, &vfr_msg);
                        MsgData.vfr.airspeed = vfr_msg.airspeed;
                        MsgData.vfr.groundspeed = vfr_msg.groundspeed;
                        MsgData.vfr.heading = vfr_msg.heading;
                        MsgData.vfr.throttle = vfr_msg.throttle;
                        MsgData.vfr.alt = vfr_msg.alt;
                        MsgData.vfr.climb = vfr_msg.climb;
                    }

              
                    if (msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) 
                    {
                          
                        mavlink_servo_output_raw_t servo_raw_msg;
                        mavlink_msg_servo_output_raw_decode(&msg, &servo_raw_msg);
                        MsgData.raw.time_usec = servo_raw_msg.time_usec;
                        // MsgData.raw.active = servo_raw_msg.active;
                        // for (int i = 0; i < 32; ++i) {

                        //     MsgData.raw.actuator.push_back(servo_raw_msg.actuator[i]);
                        // }
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_EXTENDED_SYS_STATE) 
                    {
                        mavlink_extended_sys_state_t ext_sys_msg;
                        mavlink_msg_extended_sys_state_decode(&msg, &ext_sys_msg);
                        MsgData.ext_sys.vtol_state = ext_sys_msg.vtol_state;
                        MsgData.ext_sys.landed_state = ext_sys_msg.landed_state;
                    }

    
                    if (msg.msgid == MAVLINK_MSG_ID_HOME_POSITION) 
                    {
                         
                        mavlink_home_position_t home_pos_msg;
                        mavlink_msg_home_position_decode(&msg, &home_pos_msg);
                        MsgData.home_pos.latitude = home_pos_msg.latitude;
                        MsgData.home_pos.longitude = home_pos_msg.longitude;
                        MsgData.home_pos.altitude = home_pos_msg.altitude;
                        MsgData.home_pos.x = home_pos_msg.x;
                        MsgData.home_pos.y = home_pos_msg.y;
                        MsgData.home_pos.z = home_pos_msg.z;
                        for (int i = 0; i < 4; ++i) {
                            MsgData.home_pos.q.push_back( home_pos_msg.q[i]);
                        }
                        MsgData.home_pos.approach_x = home_pos_msg.approach_x;
                        MsgData.home_pos.approach_y = home_pos_msg.approach_y;
                        MsgData.home_pos.approach_z = home_pos_msg.approach_z;
                        MsgData.home_pos.time_usec = home_pos_msg.time_usec;
                    }

                   
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) 
                    {
                       
                        mavlink_heartbeat_t heartbeat_msg;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat_msg);
                        MsgData.heartbeat.type = heartbeat_msg.type;
                        MsgData.heartbeat.autopilot = heartbeat_msg.autopilot;
                        MsgData.heartbeat.base_mode = heartbeat_msg.base_mode;
                        MsgData.heartbeat.custom_mode = heartbeat_msg.custom_mode;
                        MsgData.heartbeat.system_status = heartbeat_msg.system_status;
        
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) 
                    {
                       
                        mavlink_sys_status_t sys_status_msg;
                        mavlink_msg_sys_status_decode(&msg, &sys_status_msg);
                        MsgData.sys.onboard_control_sensors_present = sys_status_msg.onboard_control_sensors_present;
                        MsgData.sys.onboard_control_sensors_enabled = sys_status_msg.onboard_control_sensors_enabled;
                        MsgData.sys.onboard_control_sensors_health = sys_status_msg.onboard_control_sensors_health;
                        MsgData.sys.load = sys_status_msg.load;
                        MsgData.sys.voltage_battery = sys_status_msg.voltage_battery;
                        MsgData.sys.current_battery = sys_status_msg.current_battery;
                        MsgData.sys.battery_remaining = sys_status_msg.battery_remaining;
                        MsgData.sys.drop_rate_comm = sys_status_msg.drop_rate_comm;
                        MsgData.sys.errors_comm = sys_status_msg.errors_comm;
                        MsgData.sys.errors_count1 = sys_status_msg.errors_count1;
                        MsgData.sys.errors_count2 = sys_status_msg.errors_count2;
                        MsgData.sys.errors_count3 = sys_status_msg.errors_count3;
                        MsgData.sys.errors_count4 = sys_status_msg.errors_count4;
                        MsgData.sys.onboard_control_sensors_present_extended = sys_status_msg.onboard_control_sensors_present_extended;
                        MsgData.sys.onboard_control_sensors_enabled_extended = sys_status_msg.onboard_control_sensors_enabled_extended;
                        MsgData.sys.onboard_control_sensors_health_extended = sys_status_msg.onboard_control_sensors_health_extended;
                    }


                    if (msg.msgid == MAVLINK_MSG_ID_SCALED_IMU) 
                    {
                      
                        mavlink_scaled_imu_t imu_msg;
                        mavlink_msg_scaled_imu_decode(&msg, &imu_msg);
                        MsgData.imu.time_boot_ms = imu_msg.time_boot_ms;
                        MsgData.imu.xacc = imu_msg.xacc;
                        MsgData.imu.yacc = imu_msg.yacc;
                        MsgData.imu.zacc = imu_msg.zacc;
                        MsgData.imu.xgyro = imu_msg.xgyro;
                        MsgData.imu.ygyro = imu_msg.ygyro;
                        MsgData.imu.zgyro = imu_msg.zgyro;
                        MsgData.imu.xmag = imu_msg.xmag;
                        MsgData.imu.ymag = imu_msg.ymag;
                        MsgData.imu.zmag = imu_msg.zmag;
                        MsgData.imu.temperature = imu_msg.temperature;
                    }

                 
                    if (msg.msgid == MAVLINK_MSG_ID_ODOMETRY) 
                    {
                       
                        mavlink_odometry_t odom_msg;
                        mavlink_msg_odometry_decode(&msg, &odom_msg);
                        MsgData.odometry.time_usec = odom_msg.time_usec;
                        MsgData.odometry.frame_id = odom_msg.frame_id;
                        MsgData.odometry.child_frame_id = odom_msg.child_frame_id;
                        MsgData.odometry.x = odom_msg.x;
                        MsgData.odometry.y = odom_msg.y;
                        MsgData.odometry.z = odom_msg.z;
                        
                        for (int i=0;i<4;++i)
                        {
                            MsgData.odometry.q.push_back(odom_msg.q[i]);
                        }
                    
                        
                        MsgData.odometry.vx = odom_msg.vx;
                        MsgData.odometry.vy = odom_msg.vy;
                        MsgData.odometry.vz = odom_msg.vz;
                        MsgData.odometry.rollspeed = odom_msg.rollspeed;
                        MsgData.odometry.pitchspeed = odom_msg.pitchspeed;
                        MsgData.odometry.yawspeed = odom_msg.yawspeed;
                        
                     for (int i=0;i<21;++i)
                        {
                            MsgData.odometry.pose_covariance.push_back(odom_msg.pose_covariance[i]);
                        }
                        for (int i=0;i<21;++i)
                        {
                            MsgData.odometry.velocity_covariance.push_back(odom_msg.velocity_covariance[i]);
                        }
                 
                        
                        MsgData.odometry.reset_counter = odom_msg.reset_counter;
                        MsgData.odometry.estimator_type = odom_msg.estimator_type;
                        MsgData.odometry.quality = odom_msg.quality;
}                    

                 
                    if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) 
                    {
                         
                        mavlink_statustext_t status_text_msg;
                        mavlink_msg_statustext_decode(&msg, &status_text_msg);
                        MsgData.stat_text.severity = status_text_msg.severity;
                        for (int i=0;i<50;++i)
                        {
                            MsgData.stat_text.text.push_back(MsgData.stat_text.text[i]);
                        }
                      
                        MsgData.stat_text.text[sizeof(MsgData.stat_text.text) - 1] = '\0'; 
                        MsgData.stat_text.id = status_text_msg.id;
                        MsgData.stat_text.chunk_seq = status_text_msg.chunk_seq;
                    }

                     if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                            cout<<"ack"<<endl;
                
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);

                        cout << "Команда: " << ack.command << ", Результат: " << ack.result << endl;


                        }

                
            }
        }
        
       }
        
    }


    void PrintData()
    {
        std::cout<< "Широта: " <<MsgData.position.lat/1e7<<", Долгота: "<<MsgData.position.lon/1e7<<std::endl;
    }


    
    void takeoff() {
    sleep(2);
    bool fl = true;
        uint8_t buffer[BUFFER_LENGTH];
           
      while(true)
      {
        ssize_t num_bytes = recv(udp_socket_, buffer, sizeof(buffer), 0);
        if (num_bytes <= 0) 
        {
            std::cerr << "Ошибка приема данных\n";
            return;
        }

        mavlink_message_t msg, msg1;
        mavlink_status_t status;
    
        for (ssize_t i = 0; i < num_bytes; ++i) 
        {
            
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg1, &status)&&fl) 
            {
          
       
                float lat = MsgData.position.lat/1e7;
                float lon = MsgData.position.lon/1e7;

                        printf(
                "Received message %d from %d/%d\n lon=%f lat=%f \n",
                msg1.msgid, msg1.sysid, msg1.compid, lon,lat );

                mavlink_msg_command_int_pack(42,MAV_COMP_ID_MISSIONPLANNER,&msg, msg1.sysid, msg1.compid, MAV_FRAME_GLOBAL_TERRAIN_ALT, 
                MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, MsgData.position.lat, MsgData.position.lon, 10);

                // mavlink_msg_command_long_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg, msg1.sysid, msg1.compid,MAV_CMD_NAV_TAKEOFF, 0,0 ,0,0,0,lat, lon, 10.0);


                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                
                uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                ssize_t bytes_sent = sendto(udp_socket_, buf, len, 0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
                
                for (int i =0;i<MAVLINK_MAX_PACKET_LEN;++i)
                {
                         std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
                }
                cout<<endl;
                if (bytes_sent != len) 
                {
                    std::cerr << "Ошибка отправки сообщения MAVLink\n";
                }
                cout<<"takeoff"<<endl;
               fl=false;
                }


                  
            }
      }
}


 
        

};

