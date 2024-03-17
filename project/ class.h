#include <iostream>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>
#include <atomic>
#include <functional> 
#include </home/dana/mavlink/build/include/mavlink/common/mavlink.h>

struct  MavData 
{
    mavlink_global_position_int_t pos;
    mavlink_attitude_t att;
    mavlink_heartbeat_t heartbeat;    
    mavlink_sys_status_t sys;
    mavlink_mission_item_t mis;
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
        // Создаем поток, который будет выполнять MyMethod
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
                        MsgData.pos = pos;
                    }
                
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) 
                    {
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                        MsgData.heartbeat = heartbeat;
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) 
                    {
                         mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg, &att);
                        MsgData.att = att;
                    }    

                    if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS)
                    {
                        mavlink_sys_status_t sys;
                        mavlink_msg_sys_status_decode(&msg, &sys);
                        MsgData.sys = sys;
                    }       
                    if (msg.msgid = MAVLINK_MSG_ID_MISSION_ITEM)
                    {
                        mavlink_mission_item_t mis;
                        mavlink_msg_mission_item_decode(&msg, &mis);
                        MsgData.mis = mis;
                    }            
                
            }
        }
        
       }
        
    }


    void PrintData()
    {
        std::cout<< "Широта: " <<MsgData.pos.lat/1e7<<", Долгота: "<<MsgData.pos.lon/1e7<<std::endl;
    }

};

