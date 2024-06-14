#include <sys/shm.h>

#include <iostream>
#include <string>

class SMem {
public:
    explicit SMem(const std::string &name, int id, size_t size);
    ~SMem();

    void write(const void *data, size_t size);
    void read(void *data, size_t size);
    void *get();

    void attach();
    void detach();

private:
    std::string name;
    int id;
    size_t size;

    key_t key;
    int shmid;
    void *shmp;

    bool isAttached;
};