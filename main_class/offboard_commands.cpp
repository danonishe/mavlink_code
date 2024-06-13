#include " class.h"


int MavlinkReceiver::set_offboard()
{
    mtx.lock();
    cout<<sysid<<' '<<compid<<endl;
     mavlink_message_t msg;
     offboard_ = true;
     //3576 4088
    mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg, 0,sysid,compid,
                1,
                3576, 
                0.0,
                0.0,
                0.0,
                NAN,
                NAN,
                NAN,
                NAN,
                NAN,
                NAN,
                NAN,
                NAN
            );
    int a1 = send_message(&msg);

    // mavlink_msg_command_long_pack(system_id, component_id,&msg, sysid, compid,MAV_CMD_DO_SET_MODE, 0,
    // 0, 209, 6, 0, 0, 0, 0);
     mavlink_msg_command_long_pack(system_id, component_id, &msg, sysid, compid, 
        MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0);
    int a2 = send_message(&msg);

    mavlink_message_t msg2;
    mavlink_msg_command_long_pack(system_id, component_id, &msg2, sysid, compid,MAV_CMD_COMPONENT_ARM_DISARM, 0.0,
    1.0 ,0.0,0.0,0.0,0.0,0.0, 0.0);
     int a3 = send_message(&msg2);

    mtx.unlock();
    if (a1||a2||a3) return -1;
    return 0;
}

void MavlinkReceiver::send_offboard()
{
      

    json items = j["mission"]["items"];

    double lat1 = items[0]["params"][4];
    double lon1 = items[0]["params"][5];

    double lat2 = items[1]["params"][4];
    double lon2 = items[1]["params"][5];
    cout<<lat1<<' '<<lon1<<endl;
    coordinate.getDistAndAzimut(lat1, lon1, lat2, lon2);

    coordinate.getInertialCoordinates(coordinate.dist, coordinate.azimut, MsgData.local_pos.x, MsgData.local_pos.y);
 

    float x = coordinate.x, y=coordinate.y; 
    mtx.lock();
    float z=MsgData.local_pos.z,  yaw = MsgData.att.yaw;
    int s = sysid, c = compid;
    mtx.unlock();
    cout<<x<<' '<<y<<' '<<z<<endl;
    run_offboard = true;
    float x_=0, y_=0, z_=0, yaw_=0;
    while (run_offboard)
{
    float x_=0, y_=0, z_=0, yaw_=0;
    if(offboard_)
    {
        x_=x;
        y_=y;
        z_=z;
        yaw_=yaw;
    }
    else{
        x_=0;
        y_=0;
        z_=0;
        yaw_=0;

    }
    mavlink_message_t msg;
    // mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg, 0,s,c,
    //             1,
    //             2552, 
    //             x_,
    //             y_,
    //             -z_,
    //             0.0,
    //             0.0,
    //             0.0,
    //             0.0,
    //             0.0,
    //             0.0,
    //             yaw_,
    //             0.0
    //         );
        mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg, 0,s,c,
                MAV_FRAME_LOCAL_NED,
                0b0000111111111000, 
                x_,
                y_,
                z_,
                0.0,
                0.0,
                0.0,
                NAN,
                NAN,
                NAN,
                yaw_,
                0.0
            );
        mtx.lock();
    send_message(&msg);
     mtx.unlock();
    // cout<<"send offboard"<<endl;
    sleep(0.5);
    // this_thread::sleep_for(chrono::milliseconds(500));
 }

 
}


int MavlinkReceiver::disable_offboard()
{
    mtx.lock();

    mavlink_message_t msg;
    run_offboard = false;
    offboard_ = false;
    cout<<"disable"<<endl;




    mtx.unlock();
    return 0;
}



