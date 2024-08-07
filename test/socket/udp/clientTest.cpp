#include <iostream>
#include <string>

#include "utils.hpp"

#include "socket/common.hpp"
#include "socket/udp/client.hpp"

int main()
{
    URI serverURI(getLocalIP(), 8080);
    UDPClient client;

    int status;
    std::string message;
    for (int i = 0; i < 10; i++)
    {
        status = client.sendTo("Message #" + std::to_string(i), serverURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        sleep(1000);

        URI sender;
        do
        {
            status = client.receiveFrom(message, 1024, sender);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                return 1;
            }

        } while (!SOCKET_STATUS_IS_OK(status));

        std::cout << "Received: \"" << message << "\" from " << sender << std::endl;
        message.clear();
    }
    return 0;
}