import pickle
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import torch

from lerobot.common.robot_devices.cameras.utils import Camera
from lerobot.common.robot_devices.motors.dynamixel import (
    OperatingMode,
    TorqueMode,
    convert_degrees_to_steps,
)
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError



########################################################################
# Calibration logic
########################################################################

URL_TEMPLATE = (
    "https://raw.githubusercontent.com/huggingface/lerobot/main/media/{robot}/{arm}_{position}.webp"
)

# In nominal degree range ]-180, +180[
ZERO_POSITION_DEGREE = 0
ROTATED_POSITION_DEGREE = 90
GRIPPER_OPEN_DEGREE = 60 #59.41


def assert_drive_mode(drive_mode):
    # `drive_mode` is in [0,1] with 0 means original rotation direction for the motor, and 1 means inverted.
    if not np.all(np.isin(drive_mode, [0, 1])):
        raise ValueError(f"`drive_mode` contains values other than 0 or 1: ({drive_mode})")


def apply_drive_mode(position, drive_mode):
    assert_drive_mode(drive_mode)
    # Convert `drive_mode` from [0, 1] with 0 indicates original rotation direction and 1 inverted,
    # to [-1, 1] with 1 indicates original rotation direction and -1 inverted.
    signed_drive_mode = -(drive_mode * 2 - 1)
    position *= signed_drive_mode
    return position


def reset_torque_mode(arm: MotorsBus):
    # To be configured, all servos must be in "torque disable" mode
    arm.write("Torque_Enable", TorqueMode.DISABLED.value)

    # Use 'extended position mode' for all motors except gripper, because in joint mode the servos can't
    # rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while assembling the arm,
    # you could end up with a servo with a position 0 or 4095 at a crucial point See [
    # https://emanual.robotis.com/docs/en/dxl/x/x_series/#operating-mode11]
    all_motors_except_gripper = [name for name in arm.motor_names if name != "gripper"]
    if len(all_motors_except_gripper) > 0:
        arm.write("Operating_Mode", OperatingMode.EXTENDED_POSITION.value, all_motors_except_gripper)

    # Use 'position control current based' for gripper to be limited by the limit of the current.
    # For the follower gripper, it means it can grasp an object without forcing too much even tho,
    # it's goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
    # For the leader gripper, it means we can use it as a physical trigger, since we can force with our finger
    # to make it move, and it will move back to its original target position when we release the force.
    arm.write("Operating_Mode", OperatingMode.CURRENT_CONTROLLED_POSITION.value, "gripper")


def run_arm_calibration(arm: MotorsBus, name: str, arm_type: str):
    """This function ensures that a neural network trained on data collected on a given robot
    can work on another robot. For instance before calibration, setting a same goal position
    for each motor of two different robots will get two very different positions. But after calibration,
    the two robots will move to the same position.To this end, this function computes the homing offset
    and the drive mode for each motor of a given robot.

    Homing offset is used to shift the motor position to a ]-2048, +2048[ nominal range (when the motor uses 2048 steps
    to complete a half a turn). This range is set around an arbitrary "zero position" corresponding to all motor positions
    being 0. During the calibration process, you will need to manually move the robot to this "zero position".

    Drive mode is used to invert the rotation direction of the motor. This is useful when some motors have been assembled
    in the opposite orientation for some robots. During the calibration process, you will need to manually move the robot
    to the "rotated position".

    After calibration, the homing offsets and drive modes are stored in a cache.

    Example of usage:
    ```python
    run_arm_calibration(arm, "left", "follower")
    ```
    """
    reset_torque_mode(arm)

    print(f"\nRunning calibration of {name} {arm_type}...")

    print("\nMove arm to zero position")
    print("See: " + URL_TEMPLATE.format(robot="gello_ur5", arm=arm_type, position="zero"))
    input("Press Enter to continue...")

    # We arbitrarely choosed our zero target position to be a straight horizontal position with gripper upwards and closed.
    # It is easy to identify and all motors are in a "quarter turn" position. Once calibration is done, this position will
    # corresponds to every motor angle being 0. If you set all 0 as Goal Position, the arm will move in this position.
    
    zero_position = convert_degrees_to_steps(ZERO_POSITION_DEGREE, arm.motor_models)


    def _compute_nearest_rounded_position(position, models):
        # TODO(rcadene): Rework this function since some motors cant physically rotate a quarter turn
        # (e.g. the gripper of Aloha arms can only rotate ~50 degree)
        quarter_turn_degree = 90
        quarter_turn = convert_degrees_to_steps(quarter_turn_degree, models)
        nearest_pos = np.round(position.astype(float) / quarter_turn) * quarter_turn
        return nearest_pos.astype(position.dtype)

    # Compute homing offset so that `present_position + homing_offset ~= target_position`.
    position = arm.read("Present_Position")
    position = _compute_nearest_rounded_position(position, arm.motor_models)
    homing_offset = zero_position - position

    print("\nMove arm to rotated target position")
    print("See: " + URL_TEMPLATE.format(robot="gello_ur5", arm=arm_type, position="rotated"))
    input("Press Enter to continue...")

    # The rotated target position corresponds to a rotation of a quarter turn from the zero position.
    # This allows to identify the rotation direction of each motor.
    # For instance, if the motor rotates 90 degree, and its value is -90 after applying the homing offset, then we know its rotation direction
    # is inverted. However, for the calibration being successful, we need everyone to follow the same target position.
    # Sometimes, there is only one possible rotation direction. For instance, if the gripper is closed, there is only one direction which
    # corresponds to opening the gripper. When the rotation direction is ambiguous, we arbitrarely rotate clockwise from the point of view
    # of the previous motor in the kinetic chain.
    rotated_position = convert_degrees_to_steps(ROTATED_POSITION_DEGREE, arm.motor_models)

    #通过将每个电机旋转四分之一圈来找到驱动模式。
    #驱动模式指示电机旋转方向是否应反转（=1）或不应反转（=0）。
    position = arm.read("Present_Position")
    position += homing_offset
    position = _compute_nearest_rounded_position(position, arm.motor_models)
    drive_mode = (position != rotated_position).astype(np.int32)

    # Re-compute homing offset to take into account drive mode
    position = arm.read("Present_Position")
    position = apply_drive_mode(position, drive_mode)
    position = _compute_nearest_rounded_position(position, arm.motor_models)
    homing_offset = rotated_position - position

    print("\nMove arm to rest position")
    print("See: " + URL_TEMPLATE.format(robot="gello_ur5", arm=arm_type, position="rest"))
    input("Press Enter to continue...")
    print()

    return homing_offset, drive_mode


########################################################################
# Alexander gello_ur5 robot arm
########################################################################


@dataclass
class gello_ur5RobotConfig:
    """
    Example of usage:
    ```python
    gello_ur5RobotConfig()
    ```
    """

    # Define all components of the robot
    leader_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    follower_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    cameras: dict[str, Camera] = field(default_factory=lambda: {})


class gello_ur5Robot:
    def __init__(
        self,
        config: gello_ur5RobotConfig | None = None,
        calibration_path: Path = ".cache/calibration/gello_ur5.pkl",
        **kwargs,
    ):
        if config is None:
            config = gello_ur5RobotConfig()
        # Overwrite config arguments using kwargs
        self.config = replace(config, **kwargs)
        self.calibration_path = Path(calibration_path)

        self.leader_arms = self.config.leader_arms
        self.follower_arms = self.config.follower_arms
        self.cameras = self.config.cameras
        self.is_connected = False
        self.logs = {}

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "gello_ur5Robot is already connected. Do not run `robot.connect()` twice."
            )

        if not self.leader_arms and not self.follower_arms and not self.cameras:
            raise ValueError(
                "gello_ur5Robot doesn't have any device to connect. See example of usage in docstring of the class."
            )

        # Connect the arms
        for name in self.follower_arms:
            print(f"Connecting {name} follower arm.")
            self.follower_arms[name].connect()
            print(f"Connecting {name} leader arm.")
            self.leader_arms[name].connect()

        # Reset the arms and load or run calibration
        if self.calibration_path.exists():

            with open(self.calibration_path, "rb") as f:
                calibration = pickle.load(f)
        else:
            print(f"Missing calibration file '{self.calibration_path}'. Starting calibration precedure.")
            # Run calibration process which begins by reseting all arms
            calibration = self.run_calibration()

            print(f"Calibration is done! Saving calibration file '{self.calibration_path}'")
            self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.calibration_path, "wb") as f:
                pickle.dump(calibration, f)

        #print("run_calibration :",calibration)
        # Set calibration
        
        for name in self.leader_arms:
            self.leader_arms[name].set_calibration(calibration[f"leader_{name}"])

        # Connect the cameras
        for name in self.cameras:
            self.cameras[name].connect()

        self.is_connected = True

    def run_calibration(self):
        calibration = {}

        for name in self.leader_arms:
            #homing_offset, drive_mode = run_arm_calibration(self.leader_arms[name], name, "follower")
            homing_offset = np.array([0, -4096, -0, -4096, 0, 0, 414])
            
            drive_mode = np.array([ 0, 0, 1, 0, 0, 0, 0]) #旋转方向是否应反转（=1）或不应反转（=0）。
            calibration[f"leader_{name}"] = {}
            for idx, motor_name in enumerate(self.leader_arms[name].motor_names):
                calibration[f"leader_{name}"][motor_name] = (homing_offset[idx], drive_mode[idx])

        for name in self.follower_arms:
            homing_offset = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            drive_mode = np.array([ 0, 0, 0, 0, 0, 0, 0]) 
            calibration[f"follower_{name}"] = {}
            for idx, motor_name in enumerate(self.leader_arms[name].motor_names):
                calibration[f"follower_{name}"][motor_name] = (homing_offset[idx], drive_mode[idx])
        return calibration

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "gello_ur5Robot is not connected. You need to run `robot.connect()`."
            )

        # Prepare to assign the position of the leader to the follower
        leader_pos = {}
        for name in self.leader_arms:
            before_lread_t = time.perf_counter()
            leader_pos = self.leader_arms[name].read("Present_Position")
            self.logs[f"read_leader_{name}_pos_dt_s"] = time.perf_counter() - before_lread_t

        last_dim_value = leader_pos[-1]
        mapped_value = last_dim_value / GRIPPER_OPEN_DEGREE
        mapped_value = min(max(0, mapped_value), 1)
        # 更新 action 的最后一个维度为映射后的值
        leader_pos[-1] = mapped_value
        # Send action 发送关节角
        for name in self.follower_arms:
            self.follower_arms[name].set_joint_state(leader_pos)

        # Early exit when recording data is not requested
        if not record_data:
            return

        # TODO(rcadene): Add velocity and other info
        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            before_fread_t = time.perf_counter()
            #获取当前位置
            follower_pos[name] = self.follower_arms[name].get_joint_state()
            self.logs[f"follower_pos{name}"] = follower_pos[name]

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = np.concatenate(state)

        # Create action by concatenating follower goal position
        action = leader_pos

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()

            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries and format to pytorch
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = torch.from_numpy(state)
        action_dict["action"] = torch.from_numpy(action)
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = torch.from_numpy(images[name])

        return obs_dict, action_dict

    def capture_observation(self):
        """The returned observations do not have a batch dimension."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "gello_ur5Robot is not connected. You need to run `robot.connect()`."
            )

        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            follower_pos[name] = self.follower_arms[name].get_joint_state()

        
        self.logs[f"follower_pos: {name}"] = follower_pos

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = np.concatenate(state)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = torch.from_numpy(state)
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = torch.from_numpy(images[name])
        return obs_dict

    def send_action(self, action: torch.Tensor):
        """The provided action is expected to be a vector."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "gello_ur5Robot is not connected. You need to run `robot.connect()`."
            )
        for name in self.follower_arms:
            self.follower_arms[name].set_joint_state(action.numpy())

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "gello_ur5Robot is not connected. You need to run `robot.connect()` before disconnecting."
            )

        for name in self.follower_arms:
            self.follower_arms[name].disconnect()

        for name in self.leader_arms:
            self.leader_arms[name].disconnect()

        for name in self.cameras:
            self.cameras[name].disconnect()

        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
