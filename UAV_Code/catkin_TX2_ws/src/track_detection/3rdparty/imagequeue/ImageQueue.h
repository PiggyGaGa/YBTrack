#ifndef IMAGEQUEUE_LIBRARY_H
#define IMAGEQUEUE_LIBRARY_H

#if __cplusplus < 201103L
#error "请在 C++11 或更高标准下进行编译。"
#endif

#include <cstdint>
#include <memory>

namespace cv {
    class Mat;
}


namespace imagequeue {

    /**
     * 封装图像编码格式信息。
     *
     * */
    struct ImageFormat {
        uint32_t rows;          // 图像的行数。
        uint32_t cols;          // 图像的列数。
        uint32_t pixelType;     // 图像的像素编码类型。
        size_t pixelSize;       // 图像的像素编码大小。

        /**
         * 初始化 ImageFormat 的新实例。
         *
         * */
        ImageFormat() noexcept;

        /**
         * 初始化 ImageFormat 的新实例。
         *
         * @param rows 图像的行数。
         * @param cols 图像的列数。
         * @param pixelType 图像的像素编码类型。
         * @param pixelSize 图像的像素编码大小。
         *
         * */
        ImageFormat(uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize) noexcept;

        /**
         * 从原型矩阵对象构造 ImageFormat 结构。
         *
         * */
        static ImageFormat fromPrototype(const cv::Mat &mat) noexcept;
    };

    bool operator==(const ImageFormat &lhs, const ImageFormat &rhs) noexcept;

    bool operator!=(const ImageFormat &lhs, const ImageFormat &rhs) noexcept;

    /**
     * 封装对图像队列的访问。该类的实例对象不能被复制构造或者被复制赋值。
     * 不要继承该类。
     * 该类是线程安全的。
     *
     * */
    class ImageQueue {
        class _Impl;
        std::unique_ptr<_Impl> _impl;

    public:
        ImageQueue() noexcept;

        ImageQueue(const ImageQueue &) = delete;
        ImageQueue(ImageQueue &&) noexcept;

        ~ImageQueue() noexcept;

        ImageQueue &operator=(const ImageQueue &) = delete;
        ImageQueue &operator=(ImageQueue &&) noexcept;

        /**
         * 创建一个新的图像队列。
         *
         * @param name 队列的全局唯一名称。该参数不能为空串。
         * @param capacity 队列容量。该参数不能为 0。
         * @param rows 图像的行数目。该参数不能为 0。
         * @param cols 图像的列数目。该参数不能为 0。
         * @param pixelType 图像的像素编码类型。
         * @param pixelSize 图像单个像素的编码大小。该参数不能为 0。
         *
         * @throws std::system_error
         * @throws std::invalid_argument 当队列名称已经在全局名称空间中存在且给定的图像编码参数与队列的图像编码参数冲突时抛出。
         *
         * */
        void create(std::string name, uint32_t capacity, uint32_t rows, uint32_t cols, uint32_t pixelType,
                    size_t pixelSize);

        /**
         * 创建一个新的图像队列。
         *
         * @param name 队列的全局唯一名称。该参数不能为空串。
         * @param capacity 队列容量。该参数不能为 0。
         * @param format 队列中图像的编码格式。
         *
         * @throws std::system_error
         * @throws std::invalid_argument 当队列名称已经在全局名称空间中存在且给定的图像编码参数与队列的图像编码参数冲突时抛出。
         *
         * */
        void create(std::string name, uint32_t capacity, ImageFormat format);

        /**
         * 创建一个新的图像队列。
         *
         * @param name 队列的全局唯一名称。该参数不能为空串。
         * @param capacity 队列容量。该参数不能为 0。
         * @param prototype 图像队列中图像编码格式的原型提供对象。该函数将从该原型对象中提取用于图像队列的图像编码信息。该参数不能为空矩阵。
         *
         * @throws std::system_error
         * @throws std::invalid_argument 当队列名称已经在全局名称空间中存在且给定的图像编码参数与队列的图像编码参数冲突时抛出。
         *
         * */
        void create(std::string name, uint32_t capacity, const cv::Mat &prototype);

        /**
         * 打开一个已经存在的图像队列。
         *
         * @param name 图像队列的全局唯一名称。
         *
         * @returns 队列是否存在。
         *
         * @throws std::system_error
         *
         * */
        bool open(std::string name);

        /**
         * 关闭当前对象到图像队列的绑定。
         *
         * */
        void close() noexcept;

        /**
         * 检查当前对象是否已经绑定到一个图像队列。
         *
         * */
        bool opened() const noexcept;

        /**
         * 获取图像队列中的图像数量。
         * 前置条件：当前对象已经绑定到一个图像队列。
         *
         * */
        uint32_t count() const noexcept;

        /**
         * 获取图像队列的容量。
         * 前置条件：当前对象已经绑定到一个图像队列。
         *
         * */
        uint32_t capacity() const noexcept;

        /**
         * 获取图像队列中的图像编码格式。
         * 前置条件：当前对象已经绑定到一个图像队列。
         *
         * */
        ImageFormat format() const noexcept;

        /**
         * 将给定的图像矩阵加入队列。
         * 前置条件：当前对象已经通过使用 create 成员函数打开了图像队列。
         *
         * @param mat 要加入队列的图像矩阵。
         *
         * @returns 新加入的图像的访问 ID。
         *
         * @throws std::invalid_argument 当给定的图像矩阵的编码格式与图像队列的编码格式不符时抛出。
         *
         * */
        uint32_t push(const cv::Mat &mat, const int id) const;

        /**
         * 检查队列中是否存在 ID 为给定值的图像。
         * 前置条件：当前对象已经通过使用 open 成员函数打开了图像队列。
         *
         * @param id 要查找的图像 ID。
         *
         * */
        bool contains(uint32_t id) const noexcept;

        /**
         * 查找队列中 ID 为指定值的图像。
         * 前置条件：当前对象已经通过使用 open 成员函数打开了图像队列。
         *
         * @param id 要查找的图像 ID。
         * @param mat 输出参数，存储图像的矩阵。该矩阵内原有数据将被清除。
         *
         * @returns 是否成功找到了给定的 ID。
         *
         * */
        bool find(uint32_t id, cv::Mat &mat) const noexcept;
    };

}


#endif
