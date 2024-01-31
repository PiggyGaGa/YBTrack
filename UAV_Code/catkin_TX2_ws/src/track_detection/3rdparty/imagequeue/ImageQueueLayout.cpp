//
// Created by lancern on 19-3-13.
//

#include "ImageQueueLayout.h"
#include "ResourceGuard.h"
#include "util.h"

#include <cstring>
#include <system_error>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>

#define SEM_NAME_SUFFIX     "_semlock"
#define SHM_NAME_SUFFIX     "_shm"


namespace imagequeue {

    std::string prepareSemaphoreName(const std::string &name) {
        auto result = std::string();
        if (name.empty() || name.front() != '/')
            result.push_back('/');

        result.append(name);
        result.append(SEM_NAME_SUFFIX);

        return result;
    }

    std::string prepareSharedMemoryName(const std::string &name) {
        auto result = std::string();
        if (name.empty() || name.front() != '/')
            result.push_back('/');

        result.append(name);
        result.append(SHM_NAME_SUFFIX);

        return result;
    }


    void ImageQueueLayout::setLayout(std::string name, void *addr, size_t length, sem_t *syncSem) noexcept {
        _name = std::move(name);
        _addr = addr;
        _length = length;
        _syncSem = syncSem;
    }

    void *ImageQueueLayout::addr() const noexcept {
        return _addr;
    }

    QueueHead &ImageQueueLayout::head() const noexcept {
        return *reinterpret_cast<QueueHead *>(_addr);
    }

    sem_t *ImageQueueLayout::syncSem() const noexcept {
        return _syncSem;
    }

    ImageQueueLayout::ImageQueueLayout(std::string name)
        :_name(name), _addr(nullptr), _length(0), _syncSem(nullptr) {
        _syncSem = ::sem_open(prepareSemaphoreName(name).data(), 0);
        if (_syncSem == SEM_FAILED) {
            auto errorCode = errno;
            if (errorCode == ENOENT)
                throw std::runtime_error("No such image queue exist.");
            else
                util::throwSysErr(errorCode);
        }

        // Use a ResourceGuard to protect syncSem against exceptions that can be thrown later.
        auto semCloser = [] (sem_t *sem) -> void { ::sem_close(sem); };
        ResourceGuard<sem_t *, decltype(semCloser)> semGuard(_syncSem, semCloser);

        SEMAPHORE_LOCK_BEGIN(_syncSem, 0)

            auto shmFd = ::shm_open(prepareSharedMemoryName(name).data(), O_RDONLY, 0);
            if (shmFd == -1) {
                auto errorCode = errno;
                if (errorCode == ENOENT)
                    throw std::runtime_error("No such image queue exist.");
                else
                    util::throwSysErr(errorCode);
            }

            // Again use a ResourceGuard to protect shmFd against exceptions that can be thrown later.
            auto shmCloser = [] (int shmFd) -> void { ::close(shmFd); };
            ResourceGuard<int, decltype(shmCloser)> shmGuard(shmFd, shmCloser);

            _length = util::fsize(shmFd);
            if (_length == 0)
                throw std::runtime_error("Invalid shared memory segment.");

            _addr = ::mmap(nullptr, _length, PROT_READ, MAP_SHARED, shmFd, 0);
            if (_addr == MAP_FAILED)
                util::throwSysErr();

            // Close the file descriptor of the shared memory segment.
            ::close(shmFd);

            shmGuard.release();

        SEMAPHORE_LOCK_END

        semGuard.release();
    }

    ImageQueueLayout::~ImageQueueLayout() noexcept {
        ::munmap(_addr, _length);
        ::sem_close(_syncSem);
    }

    std::string ImageQueueLayout::name() const noexcept {
        return _name;
    }

    uint32_t ImageQueueLayout::capacity() const noexcept {
        return head().capacity;
    }

    uint32_t ImageQueueLayout::count() const noexcept {
        SEMAPHORE_LOCK_BEGIN(_syncSem, 0)

            const auto &h = head();
            return std::min(h.capacity, h.nextId);

        SEMAPHORE_LOCK_END
    }

    ImageFormat ImageQueueLayout::format() const noexcept {
        const auto &h = head();
        return { h.rows, h.cols, h.pixelType, h.pixelSize };
    }

    bool ImageQueueLayout::has(uint32_t id) const noexcept {
        SEMAPHORE_LOCK_BEGIN(_syncSem, 0)

            const auto &h = head();
            return id < h.nextId && h.nextId - id <= h.capacity;

        SEMAPHORE_LOCK_END
    }

    bool ImageQueueLayout::get(uint32_t id, cv::Mat &mat) const noexcept {
        SEMAPHORE_LOCK_BEGIN(_syncSem, 0)

            const auto &h = head();
            if (id >= h.nextId || h.nextId - id > h.capacity)
                return false;

            auto slotPos = id % h.capacity;
            auto imageSize = h.imageSize;
            auto slotAddr = reinterpret_cast<unsigned char *>(_addr) + sizeof(QueueHead) + slotPos * imageSize;

            mat = cv::Mat(h.rows, h.cols, h.pixelType, slotAddr).clone();
            return true;

        SEMAPHORE_LOCK_END
    }


    ImageQueueOwnedLayout::ImageQueueOwnedLayout(std::string name, uint32_t capacity, uint32_t rows, uint32_t cols,
                                                 uint32_t pixelType, size_t pixelSize)
        : ImageQueueLayout(),
          _shmname(prepareSharedMemoryName(name)),
          _semname(prepareSemaphoreName(name)) {
        // Open semaphore.
        auto sem = ::sem_open(_semname.data(), O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP  | S_IWGRP | S_IROTH | S_IWOTH, 1);
        if (sem == SEM_FAILED)
            util::throwSysErr();

        // Using a ResourceGuard to protect sem from the potential exceptions thrown later.
        auto semCloser = [] (sem_t *s) -> void { ::sem_close(s); };
        ResourceGuard<sem_t *, decltype(semCloser)> semGuard(sem, semCloser);

        SEMAPHORE_LOCK_BEGIN(sem, 0)

            // Open shared memory.
            auto shmFd = ::shm_open(_shmname.data(),
                                    O_CREAT | O_RDWR,
                                    S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
            if (shmFd == -1)
                util::throwSysErr();

            // Again use a ResourceGuard to protect the shared memory file descriptor from the potential exceptions.
            auto shmCloser = [] (int fd) -> void { ::close(fd); };
            ResourceGuard<int, decltype(shmCloser)> shmGuard(shmFd, shmCloser);

            bool initialized = util::fsize(shmFd) > 0;

            auto qHead = makeQueueHead(capacity, rows, cols, pixelType, pixelSize);
            auto shmLength = sizeof(QueueHead) + capacity * qHead.imageSize;
            if (::ftruncate64(shmFd, shmLength) == -1)
                util::throwSysErr();

            auto addr = ::mmap(nullptr, shmLength, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0);
            if (addr == MAP_FAILED)
                util::throwSysErr();

            // Close the file descriptor of the shared memory segment.
            ::close(shmFd);

            if (!initialized) {
                // The shared memory segment has not been initialized.
                // Initialize it by writing the queue head to the front of the buffer.
                *reinterpret_cast<QueueHead *>(addr) = qHead;
            }

            setLayout(std::move(name), addr, shmLength, sem);

            shmGuard.release();

        SEMAPHORE_LOCK_END

        semGuard.release();
    }

    ImageQueueOwnedLayout::~ImageQueueOwnedLayout() noexcept {
        // Unlink public names.
        ::shm_unlink(_shmname.data());
        ::sem_unlink(_semname.data());
    }

    uint32_t ImageQueueOwnedLayout::push(const cv::Mat &mat,const int id) const noexcept {
        SEMAPHORE_LOCK_BEGIN(syncSem(), 0)

            auto &h = head();
            h.nextId = id+1;
            auto id = h.nextId;
            
            auto slotPos = id % h.capacity;
            auto dataBuffer = reinterpret_cast<unsigned char *>(addr()) + sizeof(QueueHead) + slotPos * h.imageSize;

            util::copyMat(mat, dataBuffer);

            return id;

        SEMAPHORE_LOCK_END
    }

}
