#include "smem/squeue.hpp"

SQueue::SQueue(const std::string &name, int id, size_t capacity)
: smem(name, id, sizeof(SQueue::SQueueData) + capacity * sizeof(char)), smutex(name, id+1) {
    // Attach to shared memory
    smem.attach();

    // Initialize the queue
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    queue->capacity = capacity;
    queue->size = 0;
    std::memset(queue->data, 0, capacity);
    smutex.unlock();
}

SQueue::~SQueue() { }

void SQueue::push(char *data, size_t size) {
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    if (queue->size + size > queue->capacity) {
        smutex.unlock();
        return;
    }

    std::memcpy(queue->data + queue->size, data, size);
    queue->size += size;
    smutex.unlock();
}

void SQueue::pop(char *data, size_t size) {
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    if (queue->size < size) {
        smutex.unlock();
        return;
    }

    std::memcpy(data, queue->data, size);
    queue->size -= size;
    std::memmove(queue->data, queue->data + size, queue->size);
    smutex.unlock();
}

bool SQueue::empty() {
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    bool result = queue->size == 0;
    smutex.unlock();
    return result;
}

size_t SQueue::size() {
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    size_t result = queue->size;
    smutex.unlock();
    return result;
}

size_t SQueue::capacity() {
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    size_t result = queue->capacity;
    smutex.unlock();
    return result;
}

size_t SQueue::available() {
    smutex.lock();
    SQueueData *queue = (SQueueData *)smem.get();
    size_t result = queue->capacity - queue->size;
    smutex.unlock();
    return result;
}