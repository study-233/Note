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