#include <sys/shm.h>

#include <iostream>
#include <string>
#include <pthread.h>

class SMutex {
public:
    explicit SMutex(int id);

    ~SMutex();

    void lock();

    void unlock();

private:
    key_t key;
    int shmid;
    pthread_mutex_t *mutex;
};