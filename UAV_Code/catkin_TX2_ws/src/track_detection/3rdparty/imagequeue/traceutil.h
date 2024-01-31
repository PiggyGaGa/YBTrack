//
// Created by lancern on 19-3-14.
//

#ifndef IMAGEQUEUE_TRACEUTIL_H
#define IMAGEQUEUE_TRACEUTIL_H


#ifdef TRACE_ENABLED
#include <cstdio>

#define TRACE_ARGS(format, ...)             \
    ::printf("TRACE: in %s line %d:" format "\n", __FILE__, __LINE__, __VA_ARGS__)
#define TRACE_MSG(msg)                      \
    ::printf("TRACE: in %s line %d:" msg "\n", __FILE__, __LINE__)

#else
#define TRACE_ARGS(format, ...)         ((void)0)
#define TRACE_MSG(msg)                      ((void)0)
#endif


#endif //IMAGEQUEUE_TRACEUTIL_H
