# ImageQueue README file

ImageQueue 是一个使用共享内存机制进行进程间图片数据交换的中间件。

请务必使用 C++11 或更高标准对源代码进行编译。

## 编译

在对文件 `imagequeue.tar.gz` 解包后，进入 `ImageQueue` 项目的根目录，在命令行中执行如下语句创建编译目录并切换目录：

```bash
mkdir build && cd build
```

然后在命令行中执行如下命令进行预编译：

```bash
cmake -D CMAKE_BUILD_TYPE=Release ..
```

上述的 `CMAKE_BUILD_TYPE` 的合理取值为 `Release`（发布模式）以及 `Debug`（调试模式）。请根据需要进行设置。

最后在命令行中执行如下命令进行实际编译：

```bash
make
```

## 引入

只需要在使用本中间件的源文件中添加如下包含语句即可引入中间件的所有导出组件：
```c++
#include "ImageQueue.h"
```

本中间件采取静态链接方法链接至目标程序中。

## 使用方法

### 生产者

```c++
imagequeue::ImageQueue iq = imagequeue::ImageQueue();
iq.create("QueueName", /* 编码参数。详见 ImageQueue.h */);
uint32_t id = iq.push(mat);
```

### 消费者

```c++
imagequeue::ImageQueue iq = imagequeue::ImageQueue();
iq.create("QueueName", /* 编码参数。详见 ImageQueue.h */);
if (iq.find(id, mat))
    doSomethingWithMat(mat);
else
    printf("error: no image with id %u", id);
```

