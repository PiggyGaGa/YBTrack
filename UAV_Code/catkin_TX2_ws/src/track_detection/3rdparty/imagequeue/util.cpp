//
// Created by lancern on 19-3-13.
//

#include "util.h"

#include <cassert>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include <unistd.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>


namespace imagequeue {
    namespace util {

        void throwSysErr(error_t errCode) {
            auto msg = strerror(errCode);
            throw std::system_error(errCode, std::system_category(), msg);
        }

        void throwSysErr() {
            throwSysErr(errno);
        }

        uint64_t fsize(int fd) {
            auto pos = lseek64(fd, 0, SEEK_CUR);
            if (pos == -1)
                throwSysErr();

            auto size = lseek64(fd, 0, SEEK_END);
            if (size == -1)
                throwSysErr();

            if (lseek64(fd, pos, SEEK_SET) == -1)
                throwSysErr();

            return static_cast<uint64_t>(size);
        }

        void copyContinuousMat(const cv::Mat &mat, void *buffer) noexcept {
            auto matSize = mat.rows * mat.cols * mat.elemSize();
            memcpy(buffer, mat.data, matSize);
        }

        void copyNonContinuousMat(const cv::Mat &mat, void *buffer) noexcept {
            auto rowSize = mat.cols * mat.elemSize();
            auto movableBuffer = reinterpret_cast<char *>(buffer);

            for (auto i = 0; i < mat.rows; ++i) {
                memcpy(buffer, movableBuffer, rowSize);
                movableBuffer += rowSize;
            }
        }

        void copyMat(const cv::Mat &mat, void *buffer) noexcept {
            assert(buffer);

            if (mat.isContinuous())
                copyContinuousMat(mat, buffer);
            else
                copyNonContinuousMat(mat, buffer);
        }

        void copyMat(uint32_t rows, uint32_t cols, uint32_t type, void *buffer, cv::Mat &mat) noexcept {
            mat = cv::Mat(rows, cols, type, buffer).clone();
        }
    }
}
