#include <iostream>
#include <string>
#include <memory>

#include "utils.hpp"

#include "socket/common.hpp"
#include "socket/tcp/server.hpp"

int main()
{
    TCPServer server(URI(getLocalIP(), 8080), false);
    server.bind();
    server.listen();
    std::shared_ptr<TCPConnection> connection;
    int status = server.accept(connection);
    if (SOCKET_STATUS_IS_ERROR(status))
    {
        return 1;
    }

    std::string message;
    for (int i = 0; i < 10; i++)
    {
        sleep(1000);
        status = connection->receive(message, 25);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        URI clientURI;
        status = connection->getClientURI(clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        std::cout << "Received: \"" << message << "\" from " << clientURI << std::endl;
        connection->send("Response #" + std::to_string(i));
    }

    return 0;
}