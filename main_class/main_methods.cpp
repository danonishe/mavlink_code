#include " class.h"
int MavlinkReceiver::connect()
{   
      // Open UDP socket
  socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port
  
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14540); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }

    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        printf("setsockopt error: %s\n", strerror(errno));
        return -3;
    }

   
    src_addr_len = sizeof(src_addr);
    src_addr_set = false;

    running_ = true;
      return 0;  
}



void MavlinkReceiver::disconnect() 
    {
        if (socket_fd >= 0) 
        {
            close(socket_fd);
            socket_fd = -1;
        }

        running_ = false;
        
    }


void MavlinkReceiver::ProcessData()
{

    while(running_)
    {
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes
    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);
    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    } 

    src_addr_set = true;

    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) == 1) {
            // sysid = msg.sysid;
            // compid = msg.compid;
            // printf(
            //     "Received message %d from %d/%d\n",
            //     message.msgid, message.sysid, message.compid);
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                mavlink_heartbeat_t heartbeat_msg;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat_msg);
                MsgData.heartbeat.type = heartbeat_msg.type;
                MsgData.heartbeat.autopilot = heartbeat_msg.autopilot;
                MsgData.heartbeat.base_mode = heartbeat_msg.base_mode;
                MsgData.heartbeat.custom_mode = heartbeat_msg.custom_mode;
                MsgData.heartbeat.system_status = heartbeat_msg.system_status;
                // cout<< heartbeat_msg.base_mode<<' '<<heartbeat_msg.custom_mode<<endl;
                // printf("Got heartbeat from autopilot\n");

                if (MsgData.heartbeat.custom_mode == MAV_MODE_FLAG_GUIDED_ENABLED) {
                // Дрон находится в режиме offboard
                printf("Дрон находится в режиме offboard.\n");


                break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
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
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE:
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
                break;
            }
            case MAVLINK_MSG_ID_ALTITUDE:
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
                break;
            }
            case MAVLINK_MSG_ID_BATTERY_STATUS:
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
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT:
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
                break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
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
                break;
            }
            case MAVLINK_MSG_ID_VFR_HUD:
            {
                mavlink_vfr_hud_t vfr_msg;
                mavlink_msg_vfr_hud_decode(&msg, &vfr_msg);
                MsgData.vfr.airspeed = vfr_msg.airspeed;
                MsgData.vfr.groundspeed = vfr_msg.groundspeed;
                MsgData.vfr.heading = vfr_msg.heading;
                MsgData.vfr.throttle = vfr_msg.throttle;
                MsgData.vfr.alt = vfr_msg.alt;
                MsgData.vfr.climb = vfr_msg.climb;
                break;
            }
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            {
                mavlink_servo_output_raw_t servo_raw_msg;
                mavlink_msg_servo_output_raw_decode(&msg, &servo_raw_msg);
                MsgData.raw.time_usec = servo_raw_msg.time_usec;
                // MsgData.raw.active = servo_raw_msg.active;
                // for (int i = 0; i < 32; ++i) {

                //     MsgData.raw.actuator.push_back(servo_raw_msg.actuator[i]);
                // }
                break;
            }
            case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            {
                mavlink_extended_sys_state_t ext_sys_msg;
                mavlink_msg_extended_sys_state_decode(&msg, &ext_sys_msg);
                MsgData.ext_sys.vtol_state = ext_sys_msg.vtol_state;
                MsgData.ext_sys.landed_state = ext_sys_msg.landed_state;
                break;
            }
            case MAVLINK_MSG_ID_HOME_POSITION:
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
                break;
            }
            case MAVLINK_MSG_ID_SYS_STATUS:
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
                break;
            }
            case MAVLINK_MSG_ID_SCALED_IMU:
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
                break;
            }
            case MAVLINK_MSG_ID_ODOMETRY:
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
                break;
            }
            case MAVLINK_MSG_ID_STATUSTEXT:
            {
                mavlink_statustext_t status_text_msg;
                mavlink_msg_statustext_decode(&msg, &status_text_msg);
                MsgData.stat_text.severity = status_text_msg.severity;
                cout<<status_text_msg.severity<<endl;
                for (int i=0;i<50;++i)
                {
                   
                    MsgData.stat_text.text.push_back(status_text_msg.text[i]);
                     cout<<MsgData.stat_text.text[i];
                }
                cout<<endl;
                MsgData.stat_text.text[sizeof(MsgData.stat_text.text) - 1] = '\0'; 
                MsgData.stat_text.id = status_text_msg.id;
                MsgData.stat_text.chunk_seq = status_text_msg.chunk_seq;
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_ACK:
            {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);
                const char* result_str = "";
                            switch (ack.result) {
                case MAV_RESULT_ACCEPTED:
                    result_str = "Принято и выполнено успешно";
                    break;
                case MAV_RESULT_TEMPORARILY_REJECTED:
                    result_str = "Временно отклонено";
                    break;
                case MAV_RESULT_DENIED:
                    result_str = "Отклонено";
                    break;
                case MAV_RESULT_UNSUPPORTED:
                    result_str = "Не поддерживается";
                    break;
                case MAV_RESULT_FAILED:
                    result_str = "Не удалось";
                    break;
                default:
                    result_str = "Неизвестный результат";
                    break;
                }

                 cout << "Команда: " << ack.command << ", Результат: " << result_str << endl;
                break;
            }
        
            }

            
        }
    }

    // if (src_addr_set) {
    //     thread t2([&](){send_heartbeat();});
    //     t2.join();
    //     }
    }
}
}

void MavlinkReceiver::send_heartbeat()
{
    while(1)
    {
           mavlink_message_t message;
        // const uint8_t system_id = 42;
        const uint8_t base_mode = 0;
        const uint8_t custom_mode = 0;
        mavlink_msg_heartbeat_pack_chan(
            system_id,
            component_id,
            MAVLINK_COMM_0,
            &message,
            MAV_TYPE_GENERIC,
            MAV_AUTOPILOT_GENERIC,
            base_mode,
            custom_mode,
            MAV_STATE_STANDBY);
            sleep(2);
        // this_thread::sleep_for(chrono::milliseconds(2000));

        if (send_message(&message))  printf("heatbeat sendto error: %s\n", strerror(errno));


    }
     
}
