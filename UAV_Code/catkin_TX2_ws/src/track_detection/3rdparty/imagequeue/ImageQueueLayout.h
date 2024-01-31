//
// Created by lancern on 19-3-13.
//

#ifndef IMAGEQUEUE_IMAGEQUEUELAYOUT_H
#define IMAGEQUEUE_IMAGEQUEUELAYOUT_H

#include "ImageQueue.h"
#include "QueueHead.h"
#include "SemaphoreLocker.h"


namespace imagequeue {

    /**
     * 为 ImageQueue 提供队列承载共享内存访问。
     *
     * */
    class ImageQueueLayout {
        std::string _name;          // Name of the image queue.
        void *_addr{};              // Address of the shared memory containing the queue inside the
                                    // address space of calling process.
        size_t _length{};           // Length of the shared memory segment.
        sem_t *_syncSem{};          // Synchronization semaphore of the queue.

    protected:
        ImageQueueLayout() noexcept = default;

        /**
         * 设置 Layout 参数。
         *
         * */
        void setLayout(std::string name, void *addr, size_t length, sem_t *syncSem) noexcept;

        /**
         * 获取共享内存在调用进程中的映射基地址。
         *
         * */
        void *addr() const noexcept;

        /**
         * 获取队列头部信息。
         *
         * */
        QueueHead &head() const noexcept;

        /**
         * 获取队列全局同步信号量。
         *
         * */
        sem_t *syncSem() const noexcept;

    public:
        /**
         * 初始化 ImageQueueLayout 类的新实例。
         *
         * @throws std::runtime_error No image queue with the given name was found.
         * @throws std::system_error
         *
         * */
        explicit ImageQueueLayout(std::string name);

        ImageQueueLayout(const ImageQueueLayout &) = delete;
        ImageQueueLayout(ImageQueueLayout &&another) noexcept = default;

        ImageQueueLayout &operator=(const ImageQueueLayout &) = delete;
        ImageQueueLayout &operator=(ImageQueueLayout &&another) noexcept = default;

        virtual ~ImageQueueLayout() noexcept;

        /**
         * 获取共享内存的全局标识符。
         *
         * */
        std::string name() const noexcept;

        /**
         * 获取队列容量。
         *
         * */
        uint32_t capacity() const noexcept;

        /**
         * 获取队列中已有的图像数量。
         *
         * */
        uint32_t count() const noexcept;

        /**
         * 获取队列中图像格式。
         *
         * */
        ImageFormat format() const noexcept;

        /**
         * 确定队列中是否存在给定的图像 ID。
         *
         * */
        bool has(uint32_t id) const noexcept;

        /**
         * 获取队列中指定 ID 的图片。
         *
         * @returns 队列中是否存在给定 ID 的图片。
         *
         * */
        bool get(uint32_t id, cv::Mat &mat) const noexcept;
    };


    /**
     * 提供带所有权的 ImageQueue 承载共享内存访问。
     *
     * */
    class ImageQueueOwnedLayout : public ImageQueueLayout {
        std::string _shmname;       // Name of the shared memory segment.
        std::string _semname;       // Name of the semaphore.

    public:
        /**
         * 初始化 ImageQueueOwnedLayout 类的新实例。
         *
         * @params name 队列全局名称
         * @params capacity 队列容量
         * @params rows 队列中的图像的行数
         * @params cols 队列中的图像的列数
         * @params pixelType 队列中图像的像素类型
         * @params pixelSize 队列中图像每个像素的编码大小
         *
         * @throws std::system_error
         *
         * */
        ImageQueueOwnedLayout(std::string name, uint32_t capacity, uint32_t rows,
                              uint32_t cols, uint32_t pixelType, size_t pixelSize);

        ~ImageQueueOwnedLayout() noexcept override;

        /**
         * 将给定的图片写入队列中。
         *
         * @param mat 包含图片像素数据的矩阵。
         *
         * @returns 新插入的图片在队列中的 ID。
         *
         * */
        uint32_t push(const cv::Mat &mat, const int id) const noexcept;
    };

}


#endif //IMAGEQUEUE_IMAGEQUEUELAYOUT_H
