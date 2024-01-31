//
// Created by lancern on 19-3-13.
//

#ifndef IMAGEQUEUE_UTIL_H
#define IMAGEQUEUE_UTIL_H

#include <cstdint>
#include <cstddef>
#include <errno.h>

#include <semaphore.h>


namespace cv {
    class Mat;
}


namespace imagequeue {
    namespace util {

        /**
         * 抛出 errCode 所指示的系统错误的异常封装。
         *
         * */
        void throwSysErr(error_t errCode);

        /**
         * 抛出当前 errno 所指示的系统错误的异常封装。
         *
         * */
        void throwSysErr();

        /**
         * 执行给定的清理动作，然后抛出调用该模板函数时的活动系统错误的异常封装。
         *
         * */
        template <typename Cleanup>
        void cleanupAndThrowSysErr(Cleanup cleanup) {
            auto errCode = errno;
            cleanup();
            throwSysErr(errCode);
        }

        /**
         * 获取指定文件描述符所表示的文件的字节大小。
         *
         * @param fd 文件描述符。
         *
         * @returns 文件的字节大小。
         *
         * */
        size_t fsize(int fd);

        /**
         * 将给定的 OpenCV 矩阵数据复制到给定的缓冲区中。
         *
         * @param mat 需要复制的矩阵。
         * @param buffer 缓冲区基地址。该参数不能为 nullptr。
         *
         * */
        void copyMat(const cv::Mat &mat, void *buffer) noexcept;

        /**
         * 从给定的缓冲区中拷贝出 OpenCV 矩阵的原始数据并构造矩阵对象。
         *
         * @param rows 矩阵的行数。
         * @param cols 矩阵的列数。
         * @param type 矩阵元素的类型。
         * @param pixelSize 单个像素的编码长度。
         * @param buffer 缓冲区基地址。该参数不能为 nullptr。
         * @param mat 矩阵对象。该矩阵对象中的原有数据将被清除。
         *
         * */
        void copyMat(uint32_t rows, uint32_t cols, uint32_t type, void *buffer, cv::Mat &mat) noexcept;

    }
}

#endif //IMAGEQUEUE_UTIL_H
