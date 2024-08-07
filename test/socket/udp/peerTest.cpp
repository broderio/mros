#include "utils.hpp"
#include "socket/udp/server.hpp"

int main()
{
    UDPServer server(URI(getLocalIP(), 8080));

    int status = server.bind();
    if (SOCKET_STATUS_IS_ERROR(status))
    {
        return 1;
    }

    std::string ipAddr = getLocalIP();
    URI serverURI(ipAddr, 8080);

    std::string message;
    for (int i = 0; i < 10; i++)
    {
        message = "Message #" + std::to_string(i);
        status = server.sendTo(message, serverURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            return 1;
        }

        sleep(1000);

        URI sender;
        do
        {
            status = server.receiveFrom(message, 1024, sender);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                return 1;
            }
        } while (status == SOCKET_OP_WOULD_BLOCK);

        std::cout << "Received: \"" << message << "\" from server at " << sender << std::endl;
    }
    return 0;
}