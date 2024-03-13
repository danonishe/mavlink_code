
#include </home/dana/mavlink/build/include/mavlink/common/mavlink.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include </home/dana/mavlink/build/include/mavlink/common/mavlink.h>
// #include <mavlink/v2.0/ardupilotmega/mavlink.h> // Подключаем библиотеку MAVLink

#define BUFFER_LENGTH 2041 // Длина буфера для получения данных

// Функция для инициализации UDP сокета
int initialize_udp_socket(int port);

int main() {
    // Инициализация UDP сокета
    int udp_socket = initialize_udp_socket(14540); // Порт, который использует jmavsim для отправки данных
    if (udp_socket < 0) {
        std::cerr << "Ошибка инициализации UDP сокета\n";
        return 1;
    }

    // Буфер для хранения данных
    uint8_t buffer[BUFFER_LENGTH];

    // Бесконечный цикл для приема и обработки данных
    while (true) {
        ssize_t num_bytes = recv(udp_socket, buffer, sizeof(buffer), 0);
        if (num_bytes <= 0) {
            std::cerr << "Ошибка приема данных\n";
            continue;
        }

        // Парсинг сообщения MAVLink
        mavlink_message_t msg;
        mavlink_status_t status;
        for (ssize_t i = 0; i < num_bytes; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                // Проверяем тип сообщения
                if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    std::cout << "Широта: " << pos.lat / 1e7 << ", Долгота: " << pos.lon / 1e7 << ", Направление: "<<pos.hdg/100;
                    std::cout << ", Скорость по у: " << pos.vy/100 << ", Скорость по х: " << pos.vx/100 << ", Скорость по z: " << pos.vz/100  << std::endl;
                    
                }
            }
        }
    }

    close(udp_socket); // Закрываем сокет при завершении работы
    return 0;
}

int initialize_udp_socket(int port) {
    int sockfd;
    struct sockaddr_in sockaddr;

    // Создание сокета
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        return -1;
    }

    // Заполнение структуры sockaddr
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr.sin_port = htons(port);

    // Привязываем сокет к адресу и порту
    if (bind(sockfd, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) < 0) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}