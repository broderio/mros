#include <iostream>
#include <string>

#include "utils.hpp"

#include "socket/common.hpp"
#include "socket/websocket/client.hpp"

int main()
{
    std::string ipAddr = getLocalIP();
    WSClient client(URI(ipAddr, 8080));

    int status;
    while (!client.isConnected())
    {
        status = client.connect();
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }
        sleep(1);
    }

    int res;
    std::string message;
    for (int i = 0; i < 3; i++)
    {
        status = client.send("Message #" + std::to_string(i));
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        sleep(1000);

        do
        {
            status = client.receive(message);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                return 1;
            }
        } while (!SOCKET_STATUS_IS_OK(status));

        URI serverURI;
        status = client.getServerURI(serverURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        std::cout << "Received: \"" << message << "\" from " << serverURI << std::endl;
        message.clear();
    }
    // std::cout << "Returned from main" << std::endl;
    return 0;
}