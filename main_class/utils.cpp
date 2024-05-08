#include " class.h"


void MavlinkReceiver::printBuffer(const uint8_t* buffer, uint16_t length) {
    for (uint16_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
    }
    std::cout << std::endl;
}

int MavlinkReceiver::send_message(mavlink_message_t* msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    ssize_t bytes_sent = sendto(socket_fd, buf, len, 0, (struct sockaddr*)&src_addr, src_addr_len);

    if(bytes_sent != len) return -1;
    return 0;
}


