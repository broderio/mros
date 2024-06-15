#ifndef SMUTEX_HPP
#define SMUTEX_HPP

#include <sys/shm.h>

#include <iostream>
#include <string>
#include <pthread.h>

#include "smem/smem.hpp" 

class SMutex {
public:
    explicit SMutex(const std::string &name, int id);

    ~SMutex();

    void lock();

    void unlock();

private:
    SMem smem;
    pthread_mutex_t *mutex;
};

#endif // SMUTEX_HPP