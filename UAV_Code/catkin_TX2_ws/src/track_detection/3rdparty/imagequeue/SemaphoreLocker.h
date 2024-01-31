//
// Created by lancern on 19-3-13.
//

#ifndef IMAGEQUEUE_SEMAPHORELOCKER_H
#define IMAGEQUEUE_SEMAPHORELOCKER_H

#include <semaphore.h>


namespace imagequeue {

    /**
     * 提供信号量的 RAII 包装。
     *
     * */
    class SemaphoreLocker {
        sem_t *_sem;

    public:
        /**
         * 初始化 SemaphoreLocker 类的新实例。该构造器将会在给定的信号量上阻塞等待。
         *
         * @param sem 要等待的信号量。
         *
         * */
        explicit SemaphoreLocker(sem_t *sem) noexcept;

        /**
         * 复制构造。该构造器将会等待复制源对象所包装的信号量。
         *
         * */
        SemaphoreLocker(const SemaphoreLocker &another);

        SemaphoreLocker(SemaphoreLocker &&another) noexcept;

        /**
         * 析构当前的实例对象。该析构器将会释放封装的信号量。
         *
         * */
        ~SemaphoreLocker();

        SemaphoreLocker &operator=(const SemaphoreLocker &another) = delete;
        SemaphoreLocker &operator=(SemaphoreLocker &&another) noexcept;

        /**
         * 释放当前 SemaphoreLocker 所持有的信号量。
         *
         * */
        void release() noexcept;
    };


#define SEMAPHORE_LOCK_BEGIN_VAR(semVar, semExpr)                   \
    {                                                               \
        auto semVar = SemaphoreLocker(semExpr);

#define SEMAPHORE_LOCK_BEGIN(semExpr, level)                        \
    SEMAPHORE_LOCK_BEGIN_VAR(__semaphore_lock##level, semExpr)

#define SEMAPHORE_LOCK_END                                          \
    }

}


#endif //IMAGEQUEUE_SEMUTIL_H
