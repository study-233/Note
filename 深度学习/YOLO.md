# YOLOv5

- datasets
  - winter_task
    - Annotations
    - image
    - labels
    - ImageSets
      - test
        - images
        - labels
      - train
        - images
        - labels
      - val
        - images
        - labels

- 其中`Annotations`存放VOC格式的标签，如果直接创建的是YOLO格式的标签则此文件夹非必要，直接将标签存放到`labels`文件夹中即可
- `image`文件夹中存放的是数据集中的图片内容
- `ImageSets`文件夹是将`image`文件夹中的图片以及`labels`内的标签按照train、test、val划分开来，每种存储在对应的文件夹中
- `labels`存放的是YOLO类型的标签

# 检测命令

detect命令中可使用的参数及其含义如下：
--source      	指定检测来源
--weights     	指定权重，不指定的话会使用yolov5s.pt预训练权重
--img-size    	指定推理图片分辨率，默认640，也可使用--img
--conf-thres   	指定置信度阈值，默认0.4，也可使用--conf
--iou-thres    	指定NMS(非极大值抑制)的IOU阈值，默认0.5
--device      	指定设备，如--device 0 --device 0,1,2,3 --device cpu
--classes     		只检测特定的类，如--classes 0 2 4 6 8
--project     		指定结果存放路径，默认./runs/detect/
--name      		指定结果存放名,默认exp
--view-img   		以图片形式显示结果
--save-txt     	输出标签结果(yolo格式)
--save-conf   	在输出标签结果txt中同样写入每个目标的置信度
--agnostic-nms 	使用agnostic NMS
--augment    	增强识别
--update     		更新所有模型
--exist-ok    		若重名不覆盖

# 训练命令

train命令中可使用的参数及其含义如下：
--weights		指定权重，如果不加此参数会默认使用官方预训练的yolov5s.pt
--cfg 		指定模型文件
--data		指定数据文件 
--hyp		指定超参数文件
--epochs		指训练完整数据的次数，默认300
--batch-size	指一次迭代训练的数据大小，默认16，官方推荐越大越好，用你GPU
能承受最大的。可简写为--batch
--img-size 	指定训练图片大小，默认640，可简写为--img
--name 		指定结果文件名，默认result.txt
--device 		指定训练设备，如--device 0,1,2,3
--local_rank 	分布式训练参数，不要自己修改！
--log-imgs 	W&B的图片数量，默认16，最大100
--workers 	指定dataloader的workers数量，默认8
--project 		训练结果存放目录，默认./runs/train/
--name 		训练结果存放名，默认exp