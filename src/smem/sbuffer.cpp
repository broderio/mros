#include "smem/sbuffer.hpp"

SBuffer::SBuffer(int id, size_t size) : mutex(id), key(-1), shmid(-1), data(nullptr), newMessage(false) {
    key = ftok("/tmp", id);
    if (key == -1) {
        std::cerr << "ftok failed" << std::endl;
        return;
    }

    shmid = shmget(key, size, IPC_CREAT | 0666);
    if (shmid == -1) {
        std::cerr << "shmget failed" << std::endl;
        return;
    }

    data = static_cast<char*>(shmat(shmid, nullptr, 0));
    if (data == reinterpret_cast<void*>(-1)) {
        std::cerr << "shmat failed" << std::endl;
        return;
    }
}

SBuffer::~SBuffer() {
    if (data != nullptr) {
        shmdt(data);
        shmctl(shmid, IPC_RMID, nullptr);
    }
}

void SBuffer::write(const std::string &data) {
    mutex.lock();
    std::memcpy(this->data, data.c_str(), data.size());
    mutex.unlock();
    newMessage = true;
}

std::string SBuffer::read(int bytes) {
    mutex.lock();
    std::string data_str(data, bytes);
    
    mutex.unlock();
    newMessage = false;
    return data_str;
}

bool SBuffer::hasNewMessage() {
    return newMessage;
}