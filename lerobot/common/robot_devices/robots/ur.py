from typing import Dict

import numpy as np


class URRobot():
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.101", no_gripper: bool = False):
        import rtde_control
        import rtde_receive

        try:
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
        except Exception as e:
            print(e)
            print(robot_ip)

        self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not no_gripper:

            from lerobot.common.robot_devices.robots.pgi_tools import PGIGripper
            self.gripper = PGIGripper()
            self.gripper.connect("/dev/ttyUSB0", 1, 115200)
            self.gripper.init_gripper()

            print("gripper connected")
            
            self.gripper.set_speed(100)
            self.gripper.set_force(20)

        [print("connect") for _ in range(4)]

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def get_gripper_Info(self) -> np.ndarray: #-> float:
        gripper_pos = self.gripper.read_current_position()
        #gripper_Speed = self.gripper.read_current_Speed()
        gripper_Force = self.gripper.read_current_Force()
        gripper_state = self.gripper.read_grip_state()
        # print("F: ",gripper_Force, "Pos: ",gripper_pos," State :",gripper_state)#" Speed: ",gripper_Speed,
        # print("")
        return np.array([gripper_pos / 1000, gripper_Force, gripper_state])

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.r_inter.getActualQ()
        robot_joints = np.rad2deg(robot_joints)
        if self._use_gripper:
            gripper_Info = self.get_gripper_Info()
            pos = np.append(robot_joints, gripper_Info)
        else:
            pos = robot_joints
        return pos

    def set_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0 / 500  # 2ms
        lookahead_time = 0.2
        gain = 100

        robot_joints = joint_state[:6]
        t_start = self.robot.initPeriod()
        joint_state_rad = np.radians(robot_joints)
        #print("joint_state:",joint_state)
        # print(f"角度：{robot_joints}")
        # print(f"弧度：{joint_state_rad}")
        # return

        self.robot.servoJ(
            joint_state_rad, velocity, acceleration, dt, lookahead_time, gain
        )
        if self._use_gripper:
            gripper_pos = int(joint_state[-1] * 1000)#?
            #print("gripper:",gripper_pos)
            self.gripper.set_position(gripper_pos)

        self.robot.waitPeriod(t_start)

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


    def connect(self) -> None:
        pass
        
    def disconnect(self) -> None:
        """Disconnect from the robot."""
        if self._use_gripper:
            self.gripper.disconnect()

def main():
    ur = URRobot("192.168.1.101", no_gripper=False)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
