#include " class.h"


int MavlinkReceiver::set_offboard()
{
    mtx.lock();

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, 0,sysid,compid,
                1,
                3576, 
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            );
    if (send_message(&msg)) return -1;

    mavlink_msg_command_long_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid,MAV_CMD_DO_SET_MODE, 0,
    0, 209, 6, 0, 0, 0, 0);
    if (send_message(&msg)) return -1;




    mtx.unlock();
    return 0;
}

int MavlinkReceiver::send_offboard()
{
        mtx.lock();

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, 0,sysid,compid,
                1,
                2552, 
                1,
               2,
                -3,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            );
    if (send_message(&msg)) return -1;





    mtx.unlock();
    return 0;
}
