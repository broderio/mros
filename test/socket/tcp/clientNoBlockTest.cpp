#include <iostream>
#include <string>
#include <memory>

#include "utils.hpp"

#include "socket/common.hpp"
#include "socket/tcp/client.hpp"

int main()
{
    std::string ipAddr = getLocalIP();
    TCPClient client(URI(ipAddr, 0), 8081);
    int status;
    do
    {
        status = client.connect();
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }
    } while (!SOCKET_STATUS_IS_OK(status));

    std::string message;
    for (int i = 0; i < 10; i++)
    {
        client.send("Message #" + std::to_string(i));
        sleep(1000);
        message.clear();
        while (message.empty())
        {
            status = client.receive(message, 25);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                return 1;
            }
        }

        URI serverURI;
        status = client.getServerURI(serverURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        std::cout << "Received: \"" << message << "\" from " << serverURI << std::endl;
    }
    return 0;
}