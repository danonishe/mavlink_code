#include " class.h"

int MavlinkReceiver::takeoff(float h)
{
    mtx.lock();
        //  cout<<sysid<<' '<<compid<<endl;
    mavlink_message_t msg0;
    float lat = MsgData.position.lat/1e7;
    float lon = MsgData.position.lon/1e7;
    h+=MsgData.position.alt/1000;

    mavlink_msg_command_long_pack(system_id, component_id, &msg0, sysid, compid,MAV_CMD_NAV_TAKEOFF,0,
    -1.0 ,0.0,0.0,NAN,NAN,NAN, h);
    int a1 = send_message(&msg0);
  

    mavlink_message_t msg2;
    mavlink_msg_command_long_pack(system_id, component_id, &msg2, sysid, compid,MAV_CMD_COMPONENT_ARM_DISARM, 0.0,
    1.0 ,0.0,0.0,0.0,0.0,0.0, 0.0);
     int a2 = send_message(&msg2);

   // printBuffer(buf0, MAVLINK_MAX_PACKET_LEN);
           mtx.unlock();
   if(a1||a2) return -1;

    return 0;

}

int MavlinkReceiver::land()
{
    mtx.lock();
    mavlink_message_t msg;

    float lat = MsgData.position.lat/1e7;
    float lon = MsgData.position.lon/1e7;
    float h = MsgData.position.relative_alt/1000;
    mavlink_msg_command_long_pack(system_id, component_id, &msg, sysid, compid,MAV_CMD_NAV_LAND,
     0,0 ,0,0,0,MsgData.position.lat,MsgData.position.lon, h);
    int a = send_message(&msg);
    mtx.unlock();
    if (a) return -1;
    return 0;
}



int MavlinkReceiver::return_to_lunch()
{

    mtx.lock();
    mavlink_message_t msg1;
    mavlink_msg_command_long_pack(system_id, component_id, &msg1, sysid, compid,MAV_CMD_NAV_RETURN_TO_LAUNCH,
     0,0 ,0,0,0,0, 0, 0);
    int a = send_message(&msg1);
    mtx.unlock();
    if (a) return -1;
    return 0;
}


 int MavlinkReceiver::change_speed(float speed)
{

    mtx.lock();
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(system_id, component_id, &msg, sysid, compid,MAV_CMD_DO_CHANGE_SPEED,0,
    0 ,speed,-1,0,0, 0, 0);
    int a = send_message(&msg);
    mtx.unlock();
    if (a) return -1;
    return 0;

}


