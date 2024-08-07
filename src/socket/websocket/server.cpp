#include "socket/websocket/server.hpp"

WSServer::WSServer()
    : opened(false)
{
}

WSServer::WSServer(const URI &uri, int maxConnections)
    : opened(false)
{
    open(uri, maxConnections);
}

WSServer::~WSServer()
{
    //std::cout << "WSServer::~WSServer()" << std::endl;
    close();
    //std::cout << "WSServer::~WSServer() after close" << std::endl;
}

int WSServer::open(const URI &uri, int maxConnections)
{
    //std::cout << "WSServer::open()" << std::endl;
    if (opened.load())
    {
        return SOCKET_OPENED;
    }

    server = std::make_shared<wspp_server>();
    server->init_asio();
    server->set_reuse_addr(true);
    server->clear_access_channels(websocketpp::log::alevel::all);
    server->set_error_channels(websocketpp::log::elevel::fatal);

    using websocketpp::lib::bind;
    using websocketpp::lib::placeholders::_1;
    using websocketpp::lib::placeholders::_2;
    server->set_open_handler(bind(&WSServer::onOpen, this, _1));
    server->set_close_handler(bind(&WSServer::onClose, this, _1));
    server->set_message_handler(bind(&WSServer::onMessage, this, _1, _2));

    websocketpp::lib::error_code ec;
    server->listen(websocketpp::lib::asio::ip::address::from_string(uri.ip), uri.port, ec);
    if (ec)
    {
        //std::cout << "WSServer::open() listen error" << std::endl;
        close();
        //std::cout << "WSServer::open() listen error after close" << std::endl;
        return SOCKET_LISTEN_FAILED;
    }

    server->start_accept();
    opened = true;

    thread = std::thread(&WSServer::run, this);

    //std::cout << "WSServer::open() before return" << std::endl;
    return SOCKET_OK;
}

int WSServer::accept(std::shared_ptr<WSConnection> &connection)
{
    //std::cout << "WSServer::accept()" << std::endl;
    if (!opened.load())
    {
        return SOCKET_NOT_OPENED;
    }

    connection_hdl connectionHdl;
    {
        scoped_lock lock(pendingConnectionsMutex);
        if (pendingConnections.empty())
        {
            return SOCKET_OP_WOULD_BLOCK;
        }

        connectionHdl = pendingConnections.front();
        pendingConnections.pop();
    }
    
    scoped_lock lock(connectionsMutex);
    connection = connections[connectionHdl];

    //std::cout << "WSServer::accept() before return" << std::endl;
    return SOCKET_OK;
}

void WSServer::close()
{
    //std::cout << "WSServer::close()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSServer::close() not opened" << std::endl;
        return;
    }

    opened.store(false);

    {
        scoped_lock guard(pendingConnectionsMutex);
        while (!pendingConnections.empty())
        {
            pendingConnections.pop();
        }
    }

    server->stop_listening();
    //std::cout << "WSServer::close() before join" << std::endl;
    if (thread.joinable())
    {
        thread.join();
    }
    //std::cout << "WSServer::close() after join" << std::endl;

}

bool WSServer::isOpen() const
{
    return opened.load();
}

int WSServer::pending() const
{
    scoped_lock guard(pendingConnectionsMutex);
    return pendingConnections.size();
}

int WSServer::getURI(URI &uri)
{
    if (!opened.load())
    {
        return SOCKET_NOT_OPENED;
    }

    websocketpp::lib::asio::error_code ec;
    websocketpp::lib::asio::ip::tcp::endpoint localEndpoint = server->get_local_endpoint(ec);
    if (ec)
    {
        return SOCKET_ENDPOINT_ERROR;
    }

    uri = URI(localEndpoint.address().to_string(), localEndpoint.port());
    return SOCKET_OK;
}

void WSServer::run()
{
    try
    {
        server->run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in websocket server: " << e.what() << std::endl;
    }
}

void WSServer::onOpen(connection_hdl hdl)
{
    std::shared_ptr<WSConnection> conn(new WSConnection(hdl, server));

    {
        scoped_lock guard(connectionsMutex);
        connections[hdl] = conn;
    }

    scoped_lock guard(pendingConnectionsMutex);
    pendingConnections.push(hdl);
}

void WSServer::onClose(connection_hdl hdl)
{
    //std::cout << "WSServer::onClose()" << std::endl;

    scoped_lock guard(connectionsMutex);
    if (connections.find(hdl) == connections.end())
    {
        //std::cout << "WSServer::onClose() connection not found" << std::endl;
        return;
    }
    if (!connections[hdl]->isOpen())
    {
        //std::cout << "WSServer::onClose() connection not open" << std::endl;
        return;
    }

    connections[hdl]->opened.store(false);
    connections.erase(hdl);
    //std::cout << "WSServer::onClose() after erase" << std::endl;
}

void WSServer::onMessage(connection_hdl hdl, wspp_server::message_ptr msg)
{
    //std::cout << "WSServer::onMessage()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSServer::onMessage() not opened" << std::endl;
        return;
    }

    scoped_lock guard(connectionsMutex);
    if (connections.find(hdl) == connections.end())
    {
        //std::cout << "WSServer::onMessage() connection not found" << std::endl;
        return;
    }
    connections[hdl]->pushMessage(msg->get_payload());
    //std::cout << "WSServer::onMessage() before return" << std::endl;
}

WSConnection::WSConnection()
: opened(false)
{
}

WSConnection::~WSConnection()
{
    //std::cout << "WSConnection::~WSConnection()" << std::endl;
    close();
    //std::cout << "WSConnection::~WSConnection() after closed" << std::endl;
}

int WSConnection::send(const std::string &message)
{
    //std::cout << "WSConnection::send()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSConnection::send() not opened" << std::endl;
        return SOCKET_SEND_FAILED;
    }
    
    std::shared_ptr<wspp_server> serverShared = server.lock();
    serverShared->send(connection, message, websocketpp::frame::opcode::text);

    //std::cout << "WSConnection::send() before return" << std::endl;
    return SOCKET_OK;
}

int WSConnection::receive(std::string &message)
{
    //std::cout << "WSConnection::receive()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSConnection::receive() not opened" << std::endl;
        return SOCKET_NOT_OPENED;
    }

    scoped_lock guard(messagesMutex);
    if (messages.empty())
    {
        return SOCKET_OP_WOULD_BLOCK;
    }
    message = messages.front();
    messages.pop();

    //std::cout << "WSConnection::receive() before return" << std::endl;
    return SOCKET_OK;
}

void WSConnection::close()
{
    //std::cout << "WSConnection::close()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSConnection::close() not opened" << std::endl;
        return;
    }

    std::shared_ptr<wspp_server> serverShared = server.lock();
    serverShared->close(connection, websocketpp::close::status::normal, "");
    while (!messages.empty())
    {
        messages.pop();
    }

    //std::cout << "WSConnection::close() after close" << std::endl;
}

bool WSConnection::isOpen() const
{
    return opened.load();
}

int WSConnection::getClientURI(URI &uri) const
{
    if (!opened.load())
    {
        return SOCKET_NOT_OPENED;
    }

    std::shared_ptr<wspp_server> serverShared = server.lock();
    wspp_server::connection_ptr connection = serverShared->get_con_from_hdl(this->connection);
    uri = URI(connection->get_host(), connection->get_port());
    return SOCKET_OK;
}

WSConnection::WSConnection(connection_hdl connection, std::weak_ptr<wspp_server> server)
: connection(connection), server(server), opened(true)
{
}

void WSConnection::pushMessage(const std::string &message)
{
    //std::cout << "WSConnection::pushMessage()" << std::endl;
    if (!opened.load())
    {
        //std::cout << "WSConnection::pushMessage() not opened" << std::endl;
        return;
    }

    scoped_lock guard(messagesMutex);
    messages.push(message);
    //std::cout << "WSConnection::pushMessage() before return" << std::endl;
}