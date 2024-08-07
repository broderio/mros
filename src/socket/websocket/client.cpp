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
    //std::cout << "WSClient::~WSClient()" << std::endl;
    close();
    //std::cout << "WSClient::~WSClient() after close" << std::endl;
}

int WSClient::open(const URI &uri)
{
    //std::cout << "WSClient::open()" << std::endl;
    serverURI = uri;

    client.init_asio();
    client.set_reuse_addr(true);
    client.clear_access_channels(websocketpp::log::alevel::all);
    client.set_error_channels(websocketpp::log::elevel::fatal);
    client.start_perpetual();

    using websocketpp::lib::bind;
    using websocketpp::lib::placeholders::_1;
    using websocketpp::lib::placeholders::_2;
    client.set_open_handler(bind(&WSClient::onOpen, this, _1));
    client.set_close_handler(bind(&WSClient::onClose, this, _1));
    client.set_fail_handler(bind(&WSClient::onFail, this, _1));
    client.set_message_handler(bind(&WSClient::onMessage, this, _1, _2));

    thread = std::thread(&WSClient::run, this);

    opened = true;

    //std::cout << "WSClient::open() before return" << std::endl;
    return SOCKET_OK;
}

int WSClient::connect()
{
    //std::cout << "WSClient::connect()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSClient::connect() not opened error" << std::endl;
        return SOCKET_NOT_OPENED;
    }

    if (connected.load())
    {
        //std::cout << "WSClient::connect() already connected error" << std::endl;
        return SOCKET_CONNECTED;
    }

    if (connecting.load() && (getTimeMilli() / 1000 - connectStartTime) < SOCKET_CONNECT_TIMEOUT)
    {
        //std::cout << "WSClient::connect() already connecting error" << std::endl;
        return SOCKET_CONNECTING;
    }

    websocketpp::lib::error_code ec;
    wspp_client::connection_ptr connPtr = client.get_connection("ws://"+serverURI.toString(), ec);
    if (ec)
    {
        //std::cout << "WSClient::connect() connection error" << std::endl;
        close();
        //std::cout << "WSClient::connect() connection error after close" << std::endl;
        return SOCKET_CONNECT_FAILED;
    }

    connection = connPtr->get_handle();
    client.connect(connPtr);
    connecting.store(true);
    connectStartTime = getTimeMilli() / 1000;

    //std::cout << "WSClient::connect() before return" << std::endl;
    return SOCKET_OK;
}

int WSClient::send(const std::string &message)
{
    //std::cout << "WSClient::send()" << std::endl;
    if (!connected.load())
    {
        //std::cout << "WSClient::send() not connected error" << std::endl;
        return SOCKET_NOT_CONNECTED;
    }

    websocketpp::lib::error_code ec;
    client.send(connection, message, websocketpp::frame::opcode::text, ec);
    if (ec)
    {
        //std::cout << "WSClient::send() before close" << std::endl;
        close();
        //std::cout << "WSClient::send() after close" << std::endl;
        return SOCKET_SEND_FAILED;
    }

    //std::cout << "WSClient::send() before return" << std::endl;
    return SOCKET_OK;
}

int WSClient::receive(std::string &message)
{
    //std::cout << "WSClient::receive()" << std::endl;
    if (!connected.load())
    {
        //std::cout << "WSClient::receive() not connected error" << std::endl;
        return SOCKET_NOT_CONNECTED;
    }

    scoped_lock guard(messagesMutex);
    if (messages.empty())
    {
        //std::cout << "WSClient::receive() would block" << std::endl;
        return SOCKET_OP_WOULD_BLOCK;
    }

    message = messages.front();
    messages.pop();

    //std::cout << "WSClient::receive() before return" << std::endl;
    return SOCKET_OK;
}

void WSClient::close()
{
    //std::cout << "WSClient::close()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSClient::close() not opened" << std::endl;
        return;
    }

    if (connected.load())
    {
        connected.store(false);
        client.close(connection, websocketpp::close::status::normal, "");
        {
            scoped_lock guard(messagesMutex);
            while (!messages.empty())
            {
                messages.pop();
            }
        }
    }

    client.stop_perpetual();
    //std::cout << "WSClient::close() before join" << std::endl;
    if (thread.joinable())
    {
        thread.join();
    }
    //std::cout << "WSClient::close() after join" << std::endl;
    opened.store(false);
}

bool WSClient::isOpen() const
{
    return opened.load();
}

bool WSClient::isConnected() const
{
    return connected.load();
}

int WSClient::getServerURI(URI &uri)
{
    if (!opened.load())
    {
        return SOCKET_NOT_OPENED;
    }

    wspp_client::connection_ptr connShared = client.get_con_from_hdl(connection);
    uri = URI(connShared->get_host(), connShared->get_port());
    return SOCKET_OK;
}

void WSClient::run()
{
    //std::cout << "WSClient::run()" << std::endl;
    try
    {
        client.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in websocket client: " << e.what() << std::endl;
    }
    //std::cout << "WSClient::run() before return" << std::endl;
}

void WSClient::onOpen(connection_hdl hdl)
{
    //std::cout << "WSClient::onOpen()" << std::endl;
    connecting.store(false);
    connected.store(true);
    //std::cout << "WSClient::onOpen() before return" << std::endl;
}

void WSClient::onClose(connection_hdl hdl)
{
    //std::cout << "WSClient::onClose()" << std::endl;
    connected.store(false);
    //std::cout << "WSClient::onClose() before return" << std::endl;
}

void WSClient::onMessage(connection_hdl hdl, wspp_client::message_ptr msg)
{
    //std::cout << "WSClient::onMessage()" << std::endl;
    scoped_lock guard(messagesMutex);
    messages.push(msg->get_payload());
    //std::cout << "WSClient::onMessage() before return" << std::endl;
}

void WSClient::onFail(connection_hdl hdl)
{
    //std::cout << "WSClient::onFail()" << std::endl;
    scoped_lock guard(messagesMutex);
    connected = false;
    //std::cout << "WSClient::onFail() before return" << std::endl;
}