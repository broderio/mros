#pragma once

#include <sys/shm.h>

#include <iostream>
#include <string>
#include <pthread.h>

#include "smem/smem.hpp"
#include "smem/smutex.hpp"

class SQueue {
public:
    /**
     * @brief Construct a new SQueue object
     * 
     * @param name Name of the shared memory
     * @param id ID of the shared memory
     * @param capacity Maximum number of bytes that can be in the queue
    */
    explicit SQueue(const std::string &name, int id, size_t capacity);

    ~SQueue();

    /**
     * @brief Push an element into the queue
     * 
     * @param data Element to be pushed into the queue
    */
    void push(char *data, size_t size);

    /**
     * @brief Pop an element from the queue
     * 
     * @return Element popped from the queue
    */
    void pop(char *data, size_t size);

    /**
     * @brief Check if the queue is empty
     * 
     * @return True if the queue is empty, false otherwise
    */
    bool empty();

    /**
     * @brief Get the size of the queue
     * 
     * @return Number of elements in the queue
    */
    size_t size();

    /**
     * @brief Get the maximum size of the queue
     * 
     * @return Maximum number of bytes able to be in the queue
    */
    size_t capacity();

    /**
     * @brief Get the number of elements that can be pushed into the queue
     * 
     * @return Number of elements that can be pushed into the queue
    */
    size_t available();

private:
    struct SQueueData {
        size_t capacity;
        size_t size;
        char data[];
    };

    SQueueData *queue;
    SMem smem;
    SMutex smutex;
};