//
// Created by lancern on 19-3-13.
//

#include "SemaphoreLocker.h"
#include "traceutil.h"


namespace imagequeue {

    SemaphoreLocker::SemaphoreLocker(sem_t *sem) noexcept
        : _sem(sem) {
        ::sem_wait(sem);
    }

    SemaphoreLocker::SemaphoreLocker(const SemaphoreLocker &another)
        : _sem(another._sem) {
        ::sem_wait(_sem);
    }

    SemaphoreLocker::SemaphoreLocker(SemaphoreLocker &&another) noexcept
        : _sem(another._sem) {
        another._sem = nullptr;
    }

    SemaphoreLocker::~SemaphoreLocker() {
        release();
    }

    SemaphoreLocker &SemaphoreLocker::operator=(SemaphoreLocker &&another) noexcept {
        _sem = another._sem;
        another._sem = nullptr;

        return *this;
    }

    void SemaphoreLocker::release() noexcept {
        if (_sem) {
            ::sem_post(_sem);
            _sem = nullptr;
        }
    }

}
