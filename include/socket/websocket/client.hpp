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
#include <atomic>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>

#include "utils.hpp"

#include "socket/common.hpp"

typedef websocketpp::client<websocketpp::config::asio_client> wspp_client;
typedef websocketpp::connection_hdl connection_hdl;
typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;

// Asynchronous websocket client with synchronous wrapper

class WSClient
{
public:
    WSClient();

    WSClient(const WSClient &other) = delete;

    WSClient(const URI &uri);

    ~WSClient();

    int open(const URI &uri);

    int connect();

    int send(const std::string &message); // sends message to server

    int receive(std::string &message); // pops message from messages, returns 0 if there is a message, 1 if there is no message, and -1 if the connection is closed

    void close();

    bool isOpen() const;

    bool isConnected() const;

    bool isConnecting() const;

    int getServerURI(URI &uri);

private:
    void run();

    void onOpen(connection_hdl hdl); // sets connected to true

    void onClose(connection_hdl hdl); // sets connected to false

    void onMessage(connection_hdl hdl, wspp_client::message_ptr msg); // pushes message to messages

    void onFail(connection_hdl hdl); // sets connected to false

    wspp_client client;

    connection_hdl connection;

    std::thread thread;

    URI serverURI;

    std::atomic<bool> connected;

    std::atomic<bool> opened;

    std::atomic<bool> connecting;

    int connectStartTime;

    mutable websocketpp::lib::mutex messagesMutex;
    std::queue<std::string> messages;
};