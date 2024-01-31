//
// Created by lancern on 19-3-18.
//

#ifndef IMAGEQUEUE_RESOURCEGUARD_H
#define IMAGEQUEUE_RESOURCEGUARD_H

#include <type_traits>
#include <utility>


namespace imagequeue {

    /**
     * 为需要释放操作的资源提供 RAII 包装。
     *
     * */
    template <typename T, typename Deleter>
    class ResourceGuard {
        T _value;
        Deleter _del;
        bool _rel;

    public:
        explicit ResourceGuard(T value, Deleter del = Deleter()) noexcept
            : _value(std::move(value)), _del(del), _rel(false)
        { }

        ~ResourceGuard() {
            if (_rel)
                return;

            _del(_value);
        }

        T value() const {
            return _value;
        }

        void release() {
            _rel = true;
        }
    };

}


#endif //IMAGEQUEUE_RESOURCEGUARD_H
