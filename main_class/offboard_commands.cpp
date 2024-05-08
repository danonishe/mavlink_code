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

    json items = j["mission"]["items"];

    double lat1 = items[0]["params"][4];
    double lon1 = items[0]["params"][5];

    double lat2 = items[3]["params"][4];
    double lon2 = items[3]["params"][5];

    cout<<lat1<<' '<<lon1<<' '<< lat2<<' '<<lon2<<endl;

    coordinate.getDistAndAzimut(lat1, lon1, lat2, lon2);



   
    coordinate.getInertialCoordinates(coordinate.dist, coordinate.azimut, MsgData.local_pos.x, MsgData.local_pos.y);
 

    float x = coordinate.x, y=coordinate.y, z=MsgData.local_pos.z, yaw = MsgData.att.yaw;

    cout<<x<<' '<<y<<' '<<z<<endl;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, 0,sysid,compid,
                1,
                2552, 
                x,
                y,
                -z,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                yaw,
                0.0
            );
    if (send_message(&msg)) return -1;





    mtx.unlock();
    return 0;
}
