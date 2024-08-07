#include "utils.hpp"

#include "socket/common.hpp"
#include "socket/udp/server.hpp"

int main()
{
    UDPServer server(URI(getLocalIP(), 8080));

    int status = server.bind();
    if (SOCKET_STATUS_IS_ERROR(status))
    {
        return 1;
    }

    URI clientURI;
    std::string message;
    for (int i = 0; i < 10; i++)
    {
        do
        {
            status = server.receiveFrom(message, 1024, clientURI);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                return 1;
            }
        } while (!SOCKET_STATUS_IS_OK(status));

        std::cout << "Received: \"" << message << "\" from " << clientURI << std::endl;

        status = server.sendTo(message, clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }
    }

    return 0;
}