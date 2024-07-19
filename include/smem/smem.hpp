#pragma once

#include <sys/shm.h>

#include <iostream>
#include <string>

#include "utils.hpp"

class SMem {
public:
    explicit SMem(const std::string &name, int id, size_t size);
    ~SMem();

    void *get();

    void detach();
    void attach();

private:
    std::string name;
    int id;
    size_t size;

    key_t key;
    int shmid;
    void *shmp;

    bool isAttached;
};