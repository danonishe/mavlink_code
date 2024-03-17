
#include<iostream>
#include " class.h"

int main()
 {
    MavlinkReceiver receiver;
    if (!receiver.connect(14540)) 
    {
        std::cerr << "Ошибка подключения к порту\n";
        return 1;
    }
    
    while (true)
    {
        receiver.PrintData();
    }

    return 0;
}