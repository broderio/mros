#pragma once

#include <fstream>
#include <iostream>
#include <set>
#include <vector>
#include <map>
#include <queue>
#include <streambuf>
#include <string>
#include <functional>
#include <memory>

#include "utils.hpp"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/common/thread.hpp>

typedef websocketpp::server<websocketpp::config::asio> wspp_server;
typedef websocketpp::connection_hdl connection_hdl;
typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;
typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;

// Asynchronous websocket server with synchronous wrapper

class WSConnection;

class WSServer
{
public:
    WSServer();

    WSServer(const URI &uri, int maxConnections = 10); // calls open

    ~WSServer();

    int open(const URI &uri, int maxConnections = 10); // listens, starts accepting connections, and runs the server

    int accept(std::shared_ptr<WSConnection> &connection); // removes a connection from pendingConnections and inserts it into connections

    void close();

    bool isOpen() const;

    int pending() const;
    
    URI getURI();

private:
    void run();

    void onOpen(connection_hdl hdl); // inserts connection to pendingConnections

    void onClose(connection_hdl hdl);

    void onMessage(connection_hdl connection, wspp_server::message_ptr msg);

    std::map<connection_hdl, std::shared_ptr<WSConnection>, std::owner_less<connection_hdl>> connections;

    std::shared_ptr<wspp_server> server;

    std::queue<connection_hdl> pendingConnections;

    mutable websocketpp::lib::mutex mutex;

    std::thread thread;

    bool opened;
};

class WSConnection
{
    friend class WSServer;

public:
    WSConnection();

    ~WSConnection();

    int send(const std::string &message);

    int receive(std::string &message);

    void close();

    bool isOpen() const;

    URI getClientURI() const;

private:
    WSConnection(connection_hdl connection, std::weak_ptr<wspp_server> server);

    void pushMessage(const std::string &message);

    connection_hdl connection;

    std::weak_ptr<wspp_server> server;

    mutable websocketpp::lib::mutex mutex;

    bool opened;

    std::queue<std::string> messages;
};