#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/websocket/server.hpp"

int main()
{
    WSServer server(URI(getLocalIP(), 8080));
    std::shared_ptr<WSConnection> connection;

    int status;
    do
    {
        status = server.accept(connection);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }
    } while (!SOCKET_STATUS_IS_OK(status));

    std::string message;
    for (int i = 0; i < 3; i++)
    {
        sleep(1000);

        do
        {
            status = connection->receive(message);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                return 1;
            }
        } while (!SOCKET_STATUS_IS_OK(status));

        URI clientURI;
        status = connection->getClientURI(clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        std::cout << "Received: \"" << message << "\" from " << clientURI << std::endl;
        connection->send("Response #" + std::to_string(i));
        message.clear();
    }

    // std::cout << "Returning from main" << std::endl;
    return 0;
}