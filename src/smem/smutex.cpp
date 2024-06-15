#include "smem/smutex.hpp"

SMutex::SMutex(const std::string &name, int id) 
: smem(name, id, sizeof(pthread_mutex_t)) {
    mutex = static_cast<pthread_mutex_t *>(smem.get());
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(mutex, &attr);
    pthread_mutexattr_destroy(&attr);
}

SMutex::~SMutex() {
    if (mutex != nullptr) {
        pthread_mutex_destroy(mutex);
    }
}

void SMutex::lock() {
    pthread_mutex_lock(mutex);
}

void SMutex::unlock() {
    pthread_mutex_unlock(mutex);
}