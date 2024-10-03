# 硬件清单
[Google drive](https://docs.google.com/spreadsheets/d/1t7Ry02J_dzc6oH8-CAuLcNe-Jdvoj7Wx/edit?usp=drive_link&ouid=101294868838182148795&rtpof=true&sd=true)

![Image](/image/1ur5List.png "Optional title")
![Image](/image/4material.png "Optional title")
![Image](/image/5Connectioninstructions.png "Optional title")
![Image](/image/6assemble.png "Optional title")

## 1: Dynamixel资料
[技术文档](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/)

[产品图纸](http://en.robotis.com/service/downloadpage.php?ca_id=70c0)

[相关软件](http://en.robotis.com/service/downloadpage.php?ca_id=10)

[配件选择](http://en.robotis.com/service/compatibility_table.php?cate=dx)


## 2: solidworks(2021)模型
![Image](/image/3model.png "Optional title")
[Google drive](https://drive.google.com/drive/folders/19oebxjm0MImcwCyp8crsg9qxxwVY44T8?usp=drive_link)

# 代码部分
## 1: 检查相机序号和配置文件是否对应  
![Image](/image/9canInit.png "Optional title")
```bash
python lerobot/common/robot_devices/cameras/opencv.py --fps 30 --record-time-s 3
```
## 2: 检查遥操臂的关节范围是否正常 
![Image](/image/10usbInit.png "Optional title")
```bash
python lerobot/scripts/control_robot.py teleoperate --robot-path lerobot/configs/robot/gello_ur5.yaml 
```

## 3: 遥操采集数据
```bash
python lerobot/scripts/control_robot.py record --robot-path lerobot/configs/robot/gello_ur5.yaml --repo-id cheney/dish --num-episodes 50
```

## 4: 可视化数据集
```bash
python lerobot/scripts/visualize_dataset_html.py   --root data   --repo-id cheney/dish
```

## 5: 训练
```bash
DATA_DIR=data python lerobot/scripts/train.py \
  dataset_repo_id={$HF_USER}/ur5_test\
  policy=act_ur5_real \
  env=ur5_real \
  hydra.run.dir=outputs/train/act_ur5_dish \
  hydra.job.name=act_ur5_dish \
  device=cuda \
  wandb.enable=true 
```
## 5: 评估
```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/gello_ur5.yaml \
  --fps 30 \
  --root data\
  --repo-id cheney/dish \
  --tags tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 30 \
  --reset-time-s 30 \
  --num-episodes 10 \
  -p outputs/train/act_ur5_dish/checkpoints/last/pretrained_model
```
## 6: 自定义评估
```bash
python lerobot/scripts/control_robot.py eval \
  --robot-path lerobot/configs/robot/gello_ur5.yaml \
  -p outputs/train/act_ur5_dish/checkpoints/last/pretrained_model
```
