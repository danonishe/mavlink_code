#include " class.h"

int MavlinkReceiver::parse_mission_json(string s)
{

    std::ifstream ifs("./mission_protocols/"+s);
    j = json::parse(ifs);
    ifs.close();
    return 0;
}

int MavlinkReceiver::upload_mission()
{
    mtx.lock();
    json items = j["mission"]["items"];
    int count = items.size();
    for (int k = 0;k<count;++k)
    {

        bool isSeq = 0;
        while (!isSeq)
        {
        mavlink_message_t msg;
        mavlink_msg_mission_count_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid,  count ,0,0);
        if (send_message(&msg)) return -1;

        for (int i =0;i<5&&!isSeq;++i)
        {
        char buffer[280]; 
        const int ret = recvfrom(
        socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);
        if (ret < 0) {printf("recvfrom error: %s\n", strerror(errno));} else if (ret == 0) {return -1;} 
        src_addr_set = true;
        mavlink_message_t msg;
        mavlink_status_t status;
        for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) == 1) {
                if (msg.msgid = MAVLINK_MSG_ID_MISSION_REQUEST_INT){
                fl = 1;
                mavlink_mission_request_int_t mission_request;
                mavlink_msg_mission_request_int_decode(&msg, &mission_request);
                int seq = mission_request.seq;
                if (k == seq) {isSeq = 1;cout<<"seq = "<<seq<<endl;}
                }
            }
        }
        //this_thread::sleep_for(chrono::milliseconds(1500));
        }
        }
        
        cout<<"пришло "<<k+1<<endl;
        mavlink_message_t msg_item;
        auto params = j["mission"]["items"][k]["params"];
        auto p3 = params[3];
        if(p3==nullptr)
        {
            p3 = NAN;
        }
        mavlink_msg_mission_item_pack(
            42,
            MAV_COMP_ID_MISSIONPLANNER,
            &msg_item,
            sysid,
            compid,
            k,
            items[k]["frame"],
            items[k]["command"],
            1,
            items[k]["autoContinue"],
            params[0],params[1],params[2],p3,params[4],params[5],params[6],
            0);
        if (send_message(&msg_item)) return -1;
        cout<<"ушел"<<k+1<<endl;
    }
   

        fl = 0;
        while (!fl)
        {
            char buffer[280]; 
            const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);
            if (ret < 0) {printf("recvfrom error: %s\n", strerror(errno));} else if (ret == 0) {return -1;} 
            src_addr_set = true;
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) == 1) {
                if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK){
 
                mavlink_mission_ack_t ack;
                mavlink_msg_mission_ack_decode(&msg, &ack);
                
                                const char* result_str = "";
                switch (ack.type) {
                case MAV_RESULT_ACCEPTED:
                {
                    result_str = "Принято и выполнено успешно";
                    fl = 1;
                    break;
                }
                case  MAV_MISSION_ERROR:
                 result_str = "Ошибка";
                 break;
                // case MAV_RESULT_TEMPORARILY_REJECTED:
                //     result_str = "Временно отклонено";
                //     break;
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

                 cout << ", Результат: " << result_str << endl;

                cout<<"mission ack"<<endl;
            }
        }
        }
        }

    mtx.unlock();
    return 0;
}



int MavlinkReceiver::download_mission()
{
     mtx.lock();

    fl=0;
    int count = 0;

    while(!fl)
    {
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, 0);
     if (send_message(&msg)) return -1;

    for (int i=0;i<5&&!fl;++i)
    {
        char buffer[280]; 
            const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);
            if (ret < 0) {printf("recvfrom error: %s\n", strerror(errno));} else if (ret == 0) {return -1;} 
            src_addr_set = true;
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) == 1) {
                if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT){
               
                mavlink_mission_count_t cnt;
                mavlink_msg_mission_count_decode(&msg, &cnt);
                count = cnt.count;
                cout<<"mission count "<<count<<endl;
                if (count == 7)
                {
                    fl = 1;
                    break;
                }
                 
            
                
            }
        }
        }
    }

            
    }

     mtx.unlock();
    return 0;
}


int MavlinkReceiver::set_mission(int a)
{
    mtx.lock();
    mavlink_message_t msg;
    mavlink_msg_mission_set_current_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, a);
    // mavlink_msg_command_long_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, 224, 0,
    // a, 1, 0, 0, 0, 0 , 0);

    if (send_message(&msg)) return -1;
    
    fl=0;
    while(!fl)
    {
            char buffer[280]; 
            const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);
            if (ret < 0) {printf("recvfrom error: %s\n", strerror(errno));} else if (ret == 0) {return -1;} 
            src_addr_set = true;
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) == 1) {
                if (msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT){
                fl = 1;
                mavlink_mission_current_t current;
                mavlink_msg_mission_current_decode(&msg, &current);
                int seq = current.seq;
                cout<<"mission current "<<seq<<endl;
            }
             if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT){
                fl = 1;
                mavlink_statustext_t status_text_msg;
                mavlink_msg_statustext_decode(&msg, &status_text_msg);
                uint8_t sev = status_text_msg.severity;
                // char s[50] =  status_text_msg.text;
               printf("Severity: %u, Text: %s\n", status_text_msg.severity, status_text_msg.text);
\
            }
        }
        }
    }


        mtx.unlock();
    return 0;
}



int MavlinkReceiver::start_mission()
{
    mtx.lock();
    mavlink_message_t msg;
     mavlink_msg_command_long_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, MAV_CMD_MISSION_START, 0,
    0, 0, 0, 0, 0, 0 , 0);

    if (send_message(&msg)) return -1;

    mtx.unlock();
    return 0;

}



int MavlinkReceiver::pause_mission()
{
     mtx.lock();
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, MAV_CMD_DO_SET_MODE, 0,
    209.0, 4.0, 3.0, 0, 0, 0 , 0);
    if (send_message(&msg)) return -1;

    //  mavlink_msg_command_long_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, MAV_CMD_DO_PAUSE_CONTINUE, 0,
    // a, 0, 0, 0, 0, 0 , 0);
    // mavlink_msg_command_int_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, 3, MAV_CMD_OVERRIDE_GOTO, 0, true,
    // MAV_GOTO_DO_HOLD, MAV_GOTO_HOLD_AT_CURRENT_POSITION, 0, 0, 0, 0 , 0);
    // if (send_message(&msg)) return -1;

    mtx.unlock();
    return 0;
}

int MavlinkReceiver::continue_mission()
{
     mtx.lock();
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(42,MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, MAV_CMD_DO_SET_MODE, 0,
    209.0, 4.0, 4.0, 0, 0, 0 , 0);
    if (send_message(&msg)) return -1;

    mtx.unlock();
    return 0;
}


int MavlinkReceiver::clear_mission()
{
    mtx.lock();
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack(42, MAV_COMP_ID_MISSIONPLANNER, &msg, sysid, compid, 0);
    if (send_message(&msg)) return -1;

    fl=0;
    while(!fl)
    {
            char buffer[280]; 
            const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);
            if (ret < 0) {printf("recvfrom error: %s\n", strerror(errno));} else if (ret == 0) {return -1;} 
            src_addr_set = true;
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) == 1) {
                if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK){
                fl = 1;
                mavlink_mission_ack_t ack;
                mavlink_msg_mission_ack_decode(&msg, &ack);
                cout<<"mission ack "<<endl;
                if (ack.type == MAV_MISSION_ACCEPTED)
                cout<<"success"<<endl;
            }
        }
        }
    }

    mtx.unlock();

    return 0;
}

