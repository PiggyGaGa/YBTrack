//
// Created by lancern on 19-3-13.
//

#include "ImageQueue.h"
#include "ImageQueueLayout.h"
#include "util.h"

#include <cassert>
#include <stdexcept>

#include <opencv2/opencv.hpp>


#if __cplusplus == 201103L

template <typename T, typename ...Args>
std::unique_ptr<T> __make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

#define MAKE_UNIQUE(type, ...)  ::__make_unique<type>(__VA_ARGS__)
#else
#define MAKE_UNIQUE(type, ...)  std::make_unique<type>(__VA_ARGS__)
#endif


namespace imagequeue {

    ImageFormat::ImageFormat() noexcept = default;

    ImageFormat::ImageFormat(uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize) noexcept
        : rows(rows), cols(cols), pixelType(pixelType), pixelSize(pixelSize)
    { }

    ImageFormat ImageFormat::fromPrototype(const cv::Mat &mat) noexcept {
        return { static_cast<uint32_t>(mat.rows),
                 static_cast<uint32_t>(mat.cols),
                 static_cast<uint32_t>(mat.type()),
                 static_cast<size_t>(mat.elemSize()) };
    }

    bool operator==(const ImageFormat &lhs, const ImageFormat &rhs) noexcept {
        return lhs.rows == rhs.rows &&
               lhs.cols == rhs.cols &&
               lhs.pixelType == rhs.pixelType &&
               lhs.pixelSize == rhs.pixelSize;
    }

    bool operator!=(const ImageFormat &lhs, const ImageFormat &rhs) noexcept {
        return lhs.rows != rhs.rows ||
               lhs.cols != rhs.cols ||
               lhs.pixelType != rhs.pixelType ||
               lhs.pixelSize != rhs.pixelSize;
    }


    ImageQueue::ImageQueue() noexcept = default;

    ImageQueue::ImageQueue(ImageQueue &&) noexcept = default;

    ImageQueue::~ImageQueue() noexcept = default;

    ImageQueue &ImageQueue::operator=(ImageQueue &&) noexcept = default;

    QueueHead &getQueueHead(void *baseAddr) noexcept {
        return *reinterpret_cast<QueueHead *>(baseAddr);
    }

    void *getDataAddr(void *baseAddr, uint32_t id) noexcept {
        const auto &qHead = getQueueHead(baseAddr);
        auto dataBaseAddr = reinterpret_cast<char *>(baseAddr) + sizeof(QueueHead);

        id %= qHead.capacity;
        return dataBaseAddr + id * qHead.imageSize;
    }


    class ImageQueue::_Impl {
        std::unique_ptr<ImageQueueLayout> layout;

    public:
        explicit _Impl(std::unique_ptr<ImageQueueLayout> _layout) noexcept
            : layout(std::move(_layout))
        { }

        ImageQueueLayout *getLayout() const noexcept {
            return layout.get();
        }

        ImageQueueOwnedLayout *getOwnedLayout() const noexcept {
            return dynamic_cast<ImageQueueOwnedLayout *>(layout.get());
        }
    };


    void ImageQueue::create(std::string name, uint32_t capacity,
                            uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize) {
        assert(!name.empty());
        assert(capacity);
        assert(rows);
        assert(cols);
        assert(pixelSize);

        _impl = MAKE_UNIQUE(_Impl, MAKE_UNIQUE(ImageQueueOwnedLayout, std::move(name), capacity, rows, cols, pixelType, pixelSize));
    }

    void ImageQueue::create(std::string name, uint32_t capacity, ImageFormat format) {
        create(std::move(name), capacity, format.rows, format.cols, format.pixelType, format.pixelSize);
    }

    void ImageQueue::create(std::string name, uint32_t capacity, const cv::Mat &prototype) {
        assert(!prototype.empty());
        create(std::move(name), capacity, ImageFormat::fromPrototype(prototype));
    }

    bool ImageQueue::open(std::string name) {
        try {
            _impl = MAKE_UNIQUE(_Impl, MAKE_UNIQUE(ImageQueueLayout, std::move(name)));
        } catch (std::runtime_error &) {
            return false;
        }

        return true;
    }

    void ImageQueue::close() noexcept {
        _impl.reset();
    }

    bool ImageQueue::opened() const noexcept {
        return (bool)_impl;
    }

    uint32_t ImageQueue::count() const noexcept {
        assert(opened());

        return _impl->getLayout()->count();
    }

    uint32_t ImageQueue::capacity() const noexcept {
        assert(opened());
        return _impl->getLayout()->capacity();
    }

    ImageFormat ImageQueue::format() const noexcept {
        assert(opened());
        return _impl->getLayout()->format();
    }

    uint32_t ImageQueue::push(const cv::Mat &mat,const int id) const {
        assert(opened());

        if (ImageFormat::fromPrototype(mat) != format())
            throw std::invalid_argument("Format of mat is incorrect.");

        return _impl->getOwnedLayout()->push(mat, id);
    }

    bool ImageQueue::contains(uint32_t id) const noexcept {
        assert(opened());
        return _impl->getLayout()->has(id);
    }

    bool ImageQueue::find(uint32_t id, cv::Mat &mat) const noexcept {
        assert(opened());
        return _impl->getLayout()->get(id, mat);
    }
}
