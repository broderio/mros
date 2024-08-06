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
    close();
}

int WSServer::open(const URI &uri, int maxConnections)
{
    server = std::make_shared<wspp_server>();
    server->init_asio();
    server->set_reuse_addr(true);
    server->clear_access_channels(websocketpp::log::alevel::all);

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
        return -1;
    }

    server->start_accept();
    opened = true;

    thread = std::thread(&WSServer::run, this);

    return 0;
}

int WSServer::accept(std::shared_ptr<WSConnection> &connection)
{
    scoped_lock lock(mutex);
    if (pendingConnections.empty())
    {
        return -1;
    }

    connection_hdl connectionHdl = pendingConnections.front();
    pendingConnections.pop();
    
    connection = connections[connectionHdl];
    return 0;
}

void WSServer::close()
{
    scoped_lock guard(mutex);
    opened = false;

    server->stop_listening();
    for (auto &connection : connections)
    {
        connection.second->close();
    }
    connections.clear();

    while (!pendingConnections.empty())
    {
        pendingConnections.pop();
    }

}

bool WSServer::isOpen() const
{
    scoped_lock guard(mutex);
    return opened;
}

int WSServer::pending() const
{
    scoped_lock guard(mutex);
    return pendingConnections.size();
}

URI WSServer::getURI()
{
    scoped_lock guard(mutex);
    if (!opened)
    {
        return URI();
    }

    websocketpp::lib::asio::error_code ec;
    websocketpp::lib::asio::ip::tcp::endpoint localEndpoint = server->get_local_endpoint(ec);
    if (ec)
    {
        return URI();
    }

    return URI(localEndpoint.address().to_string(), localEndpoint.port());
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
    //std::cout << "Connection opened!" << std::endl;
    scoped_lock guard(mutex);

    std::shared_ptr<WSConnection> conn(new WSConnection(hdl, server));
    connections[hdl] = conn;
    pendingConnections.push(hdl);
}

void WSServer::onClose(connection_hdl hdl)
{
    //std::cout << "Connection closed!" << std::endl;
    scoped_lock guard(mutex);
    if (!opened)
    {
        return;
    }
    connections.erase(hdl);
    opened = false;
}

void WSServer::onMessage(connection_hdl hdl, wspp_server::message_ptr msg)
{
    std::cout << "Received message!" << std::endl;
    {
        scoped_lock guard(mutex);
        if (!opened || connections.find(hdl) == connections.end())
        {
            return;
        }
    }
    std::cout << "Pushing message: " << msg->get_payload() << std::endl;
    if (connections.find(hdl) == connections.end())
    {
        std::cout << "Connection not found" << std::endl;
        return;
    }
    connections[hdl]->pushMessage(msg->get_payload());
}

WSConnection::WSConnection()
: opened(false)
{
}

WSConnection::~WSConnection()
{
    scoped_lock guard(mutex);
    if (opened)
    {
        close();
    }
}

int WSConnection::send(const std::string &message)
{
    {
        scoped_lock guard(mutex);
        if (!opened)
        {
            return -1;
        }
    }
    
    std::shared_ptr<wspp_server> serverShared = server.lock();
    serverShared->send(connection, message, websocketpp::frame::opcode::text);

    return 0;
}

int WSConnection::receive(std::string &message)
{
    scoped_lock guard(mutex);

    if (!opened)
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

void WSConnection::close()
{
    {
        scoped_lock guard(mutex);
        if (!opened)
        {
            return;
        }
    }

    std::shared_ptr<wspp_server> serverShared = server.lock();
    serverShared->close(connection, websocketpp::close::status::normal, "");
    while (!messages.empty())
    {
        messages.pop();
    }
    opened = false;
}

bool WSConnection::isOpen() const
{
    scoped_lock guard(mutex);
    return opened;
}

URI WSConnection::getClientURI() const
{
    {
        scoped_lock guard(mutex);
        if (!opened)
        {
            return URI();
        }
    }

    std::shared_ptr<wspp_server> serverShared = server.lock();
    wspp_server::connection_ptr connection = serverShared->get_con_from_hdl(this->connection);
    return URI(connection->get_host(), connection->get_port());
}

WSConnection::WSConnection(connection_hdl connection, std::weak_ptr<wspp_server> server)
: connection(connection), server(server), opened(true)
{
}

void WSConnection::pushMessage(const std::string &message)
{
    scoped_lock guard(mutex);
    messages.push(message);
    std::cout << "Message pushed" << std::endl;
}