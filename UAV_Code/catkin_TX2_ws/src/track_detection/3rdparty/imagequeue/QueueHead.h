//
// Created by lancern on 19-3-13.
//

#ifndef IMAGEQUEUE_QUEUEHEAD_H
#define IMAGEQUEUE_QUEUEHEAD_H

#include <cstddef>
#include <cstdint>


namespace imagequeue {

    /**
     * 封装在共享内存中队列的元信息。
     *
     * */
    struct QueueHead {
        uint32_t capacity;      // Number of images currently contained in the queue.
        uint32_t nextId;        // The next free image ID.
        uint32_t rows;          // Number of rows in the image matrix.
        uint32_t cols;          // Number of columns in the image matrix.
        uint32_t pixelType;     // Type of pixels in the image.
        size_t pixelSize;       // Size of each pixel in the image.
        size_t imageSize;       // Size of each image.
    };

    QueueHead makeQueueHead(uint32_t capacity, uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize);

    bool formatEqual(const QueueHead &lhs, const QueueHead &rhs) noexcept;

}


#endif //IMAGEQUEUE_QUEUEHEAD_H
