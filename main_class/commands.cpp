#include " class.h"

int MavlinkReceiver::takeoff(float h)
{
    mtx.lock();

    mavlink_message_t msg0;
    float lat = MsgData.position.lat/1e7;
    float lon = MsgData.position.lon/1e7;
    h+=MsgData.position.alt/1000;

    mavlink_msg_command_long_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg0, sysid, compid,MAV_CMD_NAV_TAKEOFF,0,
    -1.0 ,0.0,0.0,NAN,NAN,NAN, h);
    if (send_message(&msg0)) return -1;

    mavlink_message_t msg2;
    mavlink_msg_command_long_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg2, sysid, compid,MAV_CMD_COMPONENT_ARM_DISARM, 0.0,
    1.0 ,0.0,0.0,0.0,0.0,0.0, 0.0);
    if (send_message(&msg2)) return -2;

   // printBuffer(buf0, MAVLINK_MAX_PACKET_LEN);
        mtx.unlock();
    return 0;

}

int MavlinkReceiver::land()
{
    mtx.lock();
    mavlink_message_t msg;

    float lat = MsgData.position.lat/1e7;
    float lon = MsgData.position.lon/1e7;
    float h = MsgData.position.relative_alt/1000;
    mavlink_msg_command_long_pack(1, MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid,MAV_CMD_NAV_LAND,
     0,0 ,0,0,0,MsgData.position.lat,MsgData.position.lon, h);
     if (send_message(&msg)) return -1;
        return 0;
}



int MavlinkReceiver::return_to_lunch()
{

    mtx.lock();
    mavlink_message_t msg1;
    mavlink_msg_command_long_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg1, sysid, compid,MAV_CMD_NAV_RETURN_TO_LAUNCH,
     0,0 ,0,0,0,0, 0, 0);
    if (send_message(&msg1)) return -1;
    return 0;
}


 int MavlinkReceiver::change_speed(float speed)
{

    mtx.lock();
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid,MAV_CMD_DO_CHANGE_SPEED,0,
    0 ,speed,-1,0,0, 0, 0);
    if (send_message(&msg)) return -1;
    return 0;

}


