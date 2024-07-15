#pragma once

#define MEDIATOR_PORT_NUM 8000

#define MAX_MSG_SIZE 2048

#define PING_MSG 0xAB
#define PONG_MSG 0xBA

enum CORE_TOPICS
{
    SUB_REGISTER = 80,
    PUB_REGISTER,
    SUB_NOTIFY,
    PUB_NOTIFY,
    SUB_DEREGISTER,
    PUB_DEREGISTER,
    SUB_DISCONNECT,
    PUB_DISCONNECT,
    SUB_REQUEST,
    PUB_RESPONSE,
    SUB_RESPONSE,
    MED_TERMINATE
};