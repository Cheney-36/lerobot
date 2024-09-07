000000000000000000000000000000000000
python lerobot/common/robot_devices/cameras/opencv.py --fps 30 --record-time-s 3

python lerobot/scripts/control_robot.py teleoperate --robot-path lerobot/configs/robot/gello_ur5.yaml 
111111111111111111111111111111111111

python lerobot/scripts/control_robot.py record --robot-path lerobot/configs/robot/gello_ur5.yaml --repo-id cheney/cube --num-episodes 100

222222222222222222222222222222222222
python lerobot/scripts/visualize_dataset_html.py   --root data   --repo-id cheney/bottle




DATA_DIR=data python lerobot/scripts/train.py \
  dataset_repo_id={$HF_USER}/ur5_test\
  policy=act_ur5_real \
  env=ur5_real \
  hydra.run.dir=outputs/train/act_ur5_bottle \
  hydra.job.name=act_ur5_bottle \
  device=cuda \
  wandb.enable=true 



python lerobot/scripts/control_robot.py eval \
  --robot-path lerobot/configs/robot/gello_ur5.yaml \
  -p outputs/train/act_ur5_cube/checkpoints/last/pretrained_model




python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/gello_ur5.yaml \
  --fps 30 \
  --root data \
  --tags tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 30 \
  --reset-time-s 30 \
  --num-episodes 10 \
  -p outputs/train/act_ur5_bottle/checkpoints/last/pretrained_model



