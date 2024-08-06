#include "socket/websocket/client.hpp"

WSClient::WSClient()
    : opened(false), connected(false)
{
}

WSClient::WSClient(const URI &uri)
    : opened(false), connected(false)
{
    open(uri);
}

WSClient::~WSClient()
{
    close();
}

int WSClient::open(const URI &uri)
{
    serverURI = uri;

    client.init_asio();
    client.set_reuse_addr(true);
    client.clear_access_channels(websocketpp::log::alevel::all);
    client.start_perpetual();

    using websocketpp::lib::bind;
    using websocketpp::lib::placeholders::_1;
    using websocketpp::lib::placeholders::_2;
    client.set_open_handler(bind(&WSClient::onOpen, this, _1));
    client.set_close_handler(bind(&WSClient::onClose, this, _1));
    client.set_fail_handler(bind(&WSClient::onFail, this, _1));
    client.set_message_handler(bind(&WSClient::onMessage, this, _1, _2));

    // websocketpp::lib::error_code ec;
    // wspp_client::connection_ptr connPtr = client.get_connection("ws://"+serverURI.toString(), ec);
    // if (ec)
    // {
    //     return -2;
    // }

    // connection = connPtr->get_handle();
    // client.connect(connPtr);

    thread = std::thread(&WSClient::run, this);

    opened = true;
    return 0;
}

int WSClient::connect()
{
    if (!opened)
    {
        return -1;
    }

    if (connected)
    {
        return 1;
    }

    websocketpp::lib::error_code ec;
    wspp_client::connection_ptr connPtr = client.get_connection("ws://"+serverURI.toString(), ec);
    if (ec)
    {
        return -2;
    }

    connection = connPtr->get_handle();
    client.connect(connPtr);
    return 0;
}

int WSClient::send(const std::string &message)
{
    std::cout << "Attempting to send message: " << message << std::endl;

    if (!opened || !connected)
    {
        std::cout << "Failed to send message: WebSocket is not open or not connected" << std::endl;
        return -1;
    }

    websocketpp::lib::error_code ec;
    client.send(connection, message, websocketpp::frame::opcode::text, ec);
    if (ec)
    {
        std::cout << "Failed to send message: " << ec.message() << std::endl;
        return -1;
    }

    std::cout << "Message sent successfully" << std::endl;
    return 0;
}

int WSClient::receive(std::string &message)
{
    if (!opened || !connected)
    {
        return -1;
    }

    if (messages.empty())
    {
        return 1;
    }

    message = messages.front();
    messages.pop();
    return 0;
}

void WSClient::close()
{
    if (!opened || !connected)
    {
        return;
    }

    client.stop_perpetual();
    client.close(connection, websocketpp::close::status::normal, "");
    opened = false;
    connected = false;
    while (!messages.empty())
    {
        messages.pop();
    }

}

bool WSClient::isOpen() const
{
    scoped_lock guard(mutex);
    return opened;
}

bool WSClient::isConnected() const
{
    scoped_lock guard(mutex);
    return connected;
}

URI WSClient::getServerURI()
{
    if (!opened || !connected)
    {
        return URI();
    }
    wspp_client::connection_ptr connShared = client.get_con_from_hdl(connection);
    return URI(connShared->get_host(), connShared->get_port());
}

void WSClient::run()
{
    try
    {
        client.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in websocket client: " << e.what() << std::endl;
    }
}

void WSClient::onOpen(connection_hdl hdl)
{
    //std::cout << "Connection opened" << std::endl;
    scoped_lock guard(mutex);
    connected = true;
}

void WSClient::onClose(connection_hdl hdl)
{
    scoped_lock guard(mutex);
    std::cout << "Connection closed" << std::endl;
    connected = false;
}

void WSClient::onMessage(connection_hdl hdl, wspp_client::message_ptr msg)
{
    //std::cout << "Received message" << std::endl;
    scoped_lock guard(mutex);
    messages.push(msg->get_payload());
}

void WSClient::onFail(connection_hdl hdl)
{
    scoped_lock guard(mutex);
    //std::cout << "Connection failed" << std::endl;
    connected = false;
}