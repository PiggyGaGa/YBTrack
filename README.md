YBTrack是一种无人机目标跟踪的系统，算法采用YOLOv5+BYTE的组合，并在无人机视角下的数据集：Visdrone2019和MOT17数据集上进行了实验。本仓库包含所有训练的代码，跟踪代码、指标计算代码以及真机部署的代码。

## 仓库文件说明
```
Data_Config_Files-MOT17,Visdrone2019数据集的YOLO格式文件，训练YOLO的时候用
Easier_To_Use_TrackEval-直接使用开源代码[Easier_To_Use_TrackEval](https://github.com/JackWoo0831/Easier_To_Use_TrackEval)对YBTrack目标跟踪算法计算跟踪指标
Paper_Figure_Generate_Code-论文插图生成代码
UAV_Code-无人机真机部署的代码
tracker_result-YBTrack算法在Visdrone2019和MOT17数据集上的跟踪结果输出，由yolov5_tracker里的代码输出
yolov5_tracker-根据[yolov7-tracker](https://github.com/JackWoo0831/Yolov7-tracker) 适配了yolov5的YBTrack，该代码中包含对Visdrone2019和MOT17数据集处理成YOLO格式的代码，对原仓库的代码有非常小的改动。
```




## 任务说明
该文件夹用来做目标跟踪的重新训练，之前在嵌入式板子上测试了使用yolov5做目标跟踪，之前的yolov5模型是从官网直接下载的采用coco数据集训练的，现在需要对yolov5模型重新训练，获取UAV在无人机视角下的目标检测性能。

## 训练数据包括：
MOT17 VisDrone2019 两个数据集都在Windows系统下的E:\User\gyd7409\ 文件夹下

/media/rjzx/STORE/users/gyd7409/MOT17
/media/rjzx/STORE/users/gyd7409/VisDrone2019

### MOT17实验
实验步骤：
1. 利用工具把MOT17 数据集转化成yolo可以识别的格式，主要是转化标签文件
   格式转换过程
   ```
   python yolov5-tracker/tools/convert_MOT17_to_yolo.py --split train --generate_imgs
   python yolov5-tracker/tools/convert_MOT17_to_yolo.py --split val --generate_imgs
   python yolov5-tracker/tools/convert_MOT17_to_yolo.py --split test --generate_imgs
   ```
2. 加载yolo COCO预训练的模型
3. 训练MOT17的数据集
   当前的GPU设置30 batch-size， 64 batch-size 会爆炸, 大于32会莫名死机
   ```
   到github下载[yolov5](https://github.com/ultralytics/yolov5)的仓库
   cd yolov5
   python train.py --data ../Data/yaml/mot17.yaml --device 0 --epochs 200 --weights pts/yolov5s.pt --img 640 --batch-size 30
   ```
4. 在测试集中验证重新训练的结果，对比纯coco模型在测试集中的结果
   
   这里要做一些实验记录，包括：
   * 训练的损失曲线，各个指标的取值。
   * 对比在桌面端的性能和在嵌入式端的性能

5. 输出pytorch的pt模型
6. pt模型放到嵌入式板子上部署


### Visdrone2019实验

实验步骤：
1. 利用工具把VisDrone2019 数据集转化成yolo可以识别的格式，主要是转化标签文件
   格式转换过程
   ```
   python yolov5-tracker/tools/convert_VisDrone_to_yolov2.py --split VisDrone2019-MOT-train --generate_imgs
   python yolov5-tracker/tools/convert_VisDrone_to_yolov2.py --split VisDrone2019-MOT-val --generate_imgs
   python yolov5-tracker/tools/convert_VisDrone_to_yolov2.py --split VisDrone2019-MOT-test-dev --generate_imgs
   ```
2. 加载yolo COCO预训练的模型
3. 训练VisDrone的数据集
   当前的GPU只支持30 batch-size， 64 batch-size 会爆炸, 32会莫名死机
   ```
   到github下载[yolov5](https://github.com/ultralytics/yolov5)的仓库
   cd yolov5
   python train.py --data ../Data/yaml/visdrone_all.yaml --device 0 --epochs 200 --weights pts/yolov5s.pt --img 640 --batch-size 30
   ```
4. 在测试集中验证重新训练的结果，对比纯coco模型在测试集中的结果
     这里要做一些实验记录，包括：
   * 训练的损失曲线，各个指标的取值
   * 对比在桌面端的性能和在嵌入式端的性能

5. 输出pytorch的pt模型
6. pt模型放到嵌入式板子上部署


## 目标跟踪实验

首先，要根据自己的模型选择自己使用的是yolov5/yolov7/yolov8的模型，yolov7-tracker仓库默认使用的是yolov7的模型，
如果我们用yolov5的模型，需要把该仓库的tracker文件夹复制到yolov5的仓库目录下，然后才能执行跟踪。

1. 测试`visdrone2019`数据集性能

> 进入到`yolov7-tracker`目录下

> 要测试`VisDrone2019`数据集，需要指定以下参数

`--dataset visdrone` 参数是数据集所在位置
`--data_format origin` 我们已经将VisDrone2019转化为`yolo`格式了，因此，我们采用`--data_format yolo`
`--tracker bytetrack` 指定使用的跟踪模型
`--model_path` 指定模型，我们用的是yolov5 那个开源代码用的是yolov7

`--dataset_dir` 是gyd新增的参数，为了程序能够读取到其他目录下的数据
> 修改tracker/config_files/visdrone.yaml 文件里的配置
要采用tracker/track_yolov5.py 因为我们训练的是yolov5
```
python tracker/track_yolov5.py --dataset_dir /home/rjzx/gyd7409/yolo_tracker_exp/Data/ --dataset visdrone --data_format yolo --tracker bytetrack --model_path /home/rjzx/gyd7409/yolo_tracker_exp/yolov5/runs/train/yolov5s_visdrone2019_epoch200/weights/best.pt --save_videos
```



### MOT17数据集的验证

> 修改 tracker/config_files/mot17.yaml 文件配置

运行代码

```
python tracker/track_yolov5.py --dataset_dir /home/rjzx/gyd7409/yolo_tracker_exp/Data/ --dataset mot17 --data_format yolo --tracker bytetrack --model_path /home/rjzx/gyd7409/yolo_tracker_exp/yolov5/runs/train/yolov5s_mot17_epoch200/weights/best.pt --save_videos
```

### MOT17 half 数据集验证
由于MOT17数据集的test没有GT，所以要验证MOT17的跟踪性能，我们需要将train的部分数据分出来作为测试，我们把
```
MOT17-02-DPM
MOT17-04-DPM
```

```
python tracker/track_yolov5.py --dataset_dir /home/rjzx/gyd7409/yolo_tracker_exp/Data/ --dataset mot17_half --data_format yolo --tracker bytetrack --model_path /home/rjzx/gyd7409/yolo_tracker_exp/yolov5/runs/train/yolov5s_mot17_epoch200/weights/best.pt --save_videos
```
三个数据分出来放到test里做tracker的验证。

### Visdrone2019 重新调整标签后的结果

```
python tracker/track_yolov5.py --dataset_dir /home/rjzx/gyd7409/yolo_tracker_exp/Data/ --dataset visdrone_relabel --data_format yolo --tracker bytetrack --model_path /home/rjzx/gyd7409/yolo_tracker_exp/yolov5/runs/train/yolov5n_visdrone2019_gyd_epoch200/weights/best.pt --save_videos
```


bytetrack_11_01_11_35 跑的是yolov5s mot17