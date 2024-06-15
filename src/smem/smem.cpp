#include "smem/smem.hpp"

SMem::SMem(const std::string &name, int id, size_t size)
: size(size), name(name), id(id), isAttached(false) {

    // Generate a key for the shared memory segment.
    key = ftok(name.c_str(), id);
    if (key == -1) {
        // std::cerr << "ftok failed" << std::endl;
        perror("SMem error: ftok failed");
        return;
    }
    
    // Create a new segment with IPC_PRIVATE key.
    shmid = shmget(key, size, IPC_CREAT | 0666);
    if (shmid == -1) {
        perror("SMem error: shmget failed");
        return;
    }

    // Attach to the segment to get a pointer to it.
    attach();
}

SMem::~SMem() {
    if (isAttached) {
        detach();
    }

    // Deallocate the segment.
    if (shmctl(shmid, IPC_RMID, NULL) == -1) {
        if (errno != EINVAL) {
            perror("SMem error: shmctl failed");
        }
    }
}

void SMem::attach() {
    if (isAttached) {
        return;
    }

    // Attach to the segment to get a pointer to it.
    shmp = shmat(shmid, nullptr, 0);
    if (shmp == (void *) -1) {
        perror("SMem error: shmat failed");
        return;
    }
    isAttached = true;
}

void SMem::detach() {
    if (!isAttached) {
        return;
    }

    // Detach from the segment.
    if (shmdt(shmp) == -1) {
        perror("SMem error: shmdt failed");
        return;
    }
    isAttached = false;
}

void *SMem::get() {
    return shmp;
}