//
// Created by lancern on 19-3-14.
//

#include "QueueHead.h"


namespace imagequeue {

    QueueHead makeQueueHead(uint32_t capacity, uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize) {
        auto head = QueueHead();
        head.capacity = capacity;
        head.nextId = 0;

        head.rows = rows;
        head.cols = cols;
        head.pixelType = pixelType;
        head.pixelSize = pixelSize;

        head.imageSize = head.rows * head.cols * head.pixelSize;
        return head;
    }

    bool formatEqual(const QueueHead &lhs, const QueueHead &rhs) noexcept {
        return lhs.capacity == rhs.capacity &&
               lhs.rows == rhs.rows &&
               lhs.cols == rhs.cols &&
               lhs.pixelType == rhs.pixelType &&
               lhs.pixelSize == rhs.pixelSize &&
               lhs.imageSize == rhs.imageSize;
    }

}
