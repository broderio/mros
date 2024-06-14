#ifndef SARRAY_HPP
#define SARRAY_HPP

#include <sys/shm.h>

#include <iostream>
#include <string>
#include <pthread.h>

#include "smem/smutex.hpp"

class SBuffer {
public:
    explicit SBuffer(int id, size_t size);

    ~SBuffer();

    void write(const std::string &data);

    std::string read(int bytes);

    bool hasNewMessage();

private:
    SMutex mutex;
    key_t key;
    int shmid;
    char *data;
    bool newMessage;
};

#endif // SARRAY_HPP