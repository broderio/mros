#include "smem/smutex.hpp"

SMutex::SMutex(const std::string &name, int id) {

    // Generate a key for the shared memory segment.
    key = ftok(name.c_str(), id);
    if (key == -1) {
        // std::cerr << "ftok failed" << std::endl;
        perror("SMutex error: ftok failed");
        return;
    }

    // Create a new segment with IPC_PRIVATE key.
    shmid = shmget(key, sizeof(pthread_mutex_t), IPC_CREAT | 0666);
    if (shmid == -1) {
        // std::cerr << "shmget failed" << std::endl;
        perror("SMutex error: shmget failed");
        return;
    }

    mutex = static_cast<pthread_mutex_t*>(shmat(shmid, nullptr, 0));
    if (mutex == reinterpret_cast<void*>(-1)) {
        // std::cerr << "shmat failed" << std::endl;
        perror("SMutex error: shmat failed");
        return;
    }

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(mutex, &attr);
    pthread_mutexattr_destroy(&attr);
}

SMutex::~SMutex() {
    if (mutex != nullptr) {
        pthread_mutex_destroy(mutex);
        shmdt(mutex);
        shmctl(shmid, IPC_RMID, nullptr);
    }
}

void SMutex::lock() {
    pthread_mutex_lock(mutex);
}

void SMutex::unlock() {
    pthread_mutex_unlock(mutex);
}