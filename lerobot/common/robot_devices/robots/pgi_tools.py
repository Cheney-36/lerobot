from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException

import logging

class PGIGripper:
    def __init__(self):
        self.gripper = None
        self.client = None

    def connect(self, port, timeout=1, baudrate=115200, stopbits=1, bytesize=8, parity='N', slave = 1):
        self.slave = slave
        self.client = ModbusClient(port=port, timeout=timeout, baudrate=baudrate, stopbits=stopbits, bytesize=bytesize, parity=parity, )#
        # self.client.slaves = [1]
        connection = self.client.connect()
        if connection:
            # print(self.read_position())
            print("Connected to PGI gripper")
            # self.init_gripper()
            return True
        else:
            print("Failed to connect to PGI gripper")
            return False
        
    def disconnect(self):
        self.client.close()
        print("Disconnected from PGI gripper")

    def apply_action(self, position = 0, force = 0, speed = 0):
        self.set_speed(speed)
        self.set_force(force)
        self.set_position(position)

    def read_info(self):
        return self.read_force(), self.read_speed(), self.read_position(), self.read_current_position(), self.read_grip_state()

        
    ## Initialize the gripper
    def init_gripper(self):
        logging.basicConfig(level=logging.CRITICAL)
        try:
            res = self.client.write_register(0x0100, 1, slave=self.slave)
            print(res)
            # Wait for the gripper to initialize
            while True:
                status = self.client.read_holding_registers(0x0200, 1, slave=self.slave)
                # print(status)
                if status.registers[0] == 1:
                    break
            print("Gripper initialized")
        except ModbusException as e:
            print("Failed to initialize gripper")
            print(e)

    ## Calibration of the gripper
    def calibrate(self):
        try:
            self.client.write_register(0x0100, 0xA5, slave=self.slave)

            # Wait for the gripper to calibrate
            while True:
                status = self.read_initialization_state()
                if status.registers[0] == 0:
                    break
            print("Gripper calibrated")
        except ModbusException as e:
            print("Failed to calibrate gripper")
            print(e)

    # def _open(self):
    #     self.gripper = "open"

    # def close(self):
    #     self.gripper = "close"

    def set_force(self, force):
        # Ensure force is within the allowed range
        if 20 <= force <= 100:
            self.client.write_register(0x0101, force, slave=self.slave)
        else:
            print("Force value must be between 20 and 100")

    def read_force(self):
        result = self.client.read_holding_registers(0x0101, 1, slave=self.slave)
        return result.registers[0]

    def set_position(self, position):
        # Ensure position is within the allowed range
        if 0 <= position <= 1000:
            self.client.write_register(0x0103, position, slave=self.slave)
        else:
            print("Position value must be between 0 and 1000")

    def read_position(self):
        result = self.client.read_holding_registers(0x0103, 1, slave=self.slave)
        return result.registers[0]

    def set_speed(self, speed):
        # Ensure speed is within the allowed range
        if 1 <= speed <= 100:
            self.client.write_register(0x0104, speed)
        else:
            print("Speed value must be between 1 and 100")

    def read_speed(self):
        result = self.client.read_holding_registers(0x0104, 1, slave=self.slave)
        return result.registers[0]

    def read_initialization_state(self):
        result = self.client.read_holding_registers(0x0200, 1, slave=self.slave)
        return result.registers[0]
    # ·00 ：夹爪处于正在运动状态。
    # ·01 ：夹爪停止运动，且夹爪未检测到夹到物体。
    # ·02 ：夹爪停止运动，且夹爪检测到夹到物体。
    # ·03 ：夹爪检测到夹住物体后，发现物体掉落。
    # 注：如果夹爪在到达指定位置前夹住物体，那么此时也认为夹爪已经夹住物体（反馈为：02）
    def read_grip_state(self):
        result = self.client.read_holding_registers(0x0201, 1, slave=self.slave)
        return result.registers[0]

    def read_current_position(self):
        result = self.client.read_holding_registers(0x0202, 1, slave=self.slave)
        return result.registers[0]

    def read_current_Speed(self):
        result = self.client.read_holding_registers(0x0203, 1, slave=self.slave)
        return result.registers[0]
    
    def convert_to_short(self,value):
    # 确保输入值在 0 到 65535 范围内
        if not (0 <= value <= 65535):
            raise ValueError("Input value must be in the range 0 to 65535")
    # 将值映射到 -32768 到 32767 的范围
        return value - 65536 if value > 32767 else value
    
    def read_current_Force(self):
        result = self.client.read_holding_registers(0x0204,1,slave=self.slave)
        return self.convert_to_short(result.registers[0])

    def write_to_flash(self, save):
        if save in [0, 1]:
            self.client.write_register(0x0300, save, slave=self.slave)
        else:
            print("Invalid save parameter. Use 0 for default, 1 to save all parameters to flash.")

    def set_initialization_direction(self, direction):
        if direction in [0, 1]:
            self.client.write_register(0x0301, direction, slave=self.slave)
        else:
            print("Invalid direction. Use 0 for open, 1 for close.")

    def read_initialization_direction(self):
        result = self.client.read_holding_registers(0x0301, 1, slave=self.slave)
        return result.registers[0]

    def set_device_id(self, device_id):
        if 1 <= device_id <= 255:
            self.client.write_register(0x0302, device_id, slave=self.slave)
        else:
            print("Device ID must be between 1 and 255.")

    def read_device_id(self):
        result = self.client.read_holding_registers(0x0302, 1, slave=self.slave)
        return result.registers[0]

    def set_baud_rate(self, baud_rate_index):
        if 0 <= baud_rate_index <= 5:
            self.client.write_register(0x0303, baud_rate_index, slave=self.slave)
        else:
            print("Invalid baud rate index. Use 0-5 for 115200, 57600, 38400, 19200, 9600, 4800 respectively.")

    def read_baud_rate(self):
        result = self.client.read_holding_registers(0x0303, 1, slave=self.slave)
        return result.registers[0]

    def set_stop_bits(self, stop_bits):
        if stop_bits in [0, 1]:
            self.client.write_register(0x0304, stop_bits, slave=self.slave)
        else:
            print("Invalid stop bits. Use 0 for 1 stop bit, 1 for 2 stop bits.")

    def read_stop_bits(self):
        result = self.client.read_holding_registers(0x0304, 1, slave=self.slave)
        return result.registers[0]

    def set_parity(self, parity):
        if parity in [0, 1, 2]:
            self.client.write_register(0x0305, parity, slave=self.slave)
        else:
            print("Invalid parity. Use 0 for none, 1 for odd, 2 for even.")

    def read_parity(self):
        result = self.client.read_holding_registers(0x0305, 1, slave=self.slave)
        return result.registers[0]

    def test_io_parameters(self, io_function):
        if 1 <= io_function <= 4:
            self.client.write_register(0x0400, io_function, slave=self.slave)
        else:
            print("Invalid IO function. Use 1-4.")

    def set_io_mode(self, mode):
        if mode in [0, 1]:
            self.client.write_register(0x0402, mode, slave=self.slave)
        else:
            print("Invalid IO mode. Use 0 for disable, 1 for enable.")

    def read_io_mode(self):
        result = self.client.read_holding_registers(0x0402, 1, slave=self.slave)
        return result.registers[0]

    def set_io_level(self, io_number, level):
        if io_number in [0x0403, 0x0404] and level in [0, 1]:
            self.client.write_register(io_number, level, slave=self.slave)
        else:
            print("Invalid IO number or level. Use 0x0403 or 0x0404 for IO number, and 0 for 0V, 1 for 24V.")

    def read_io_level(self, io_number):
        if io_number in [0x0403, 0x0404]:
            result = self.client.read_holding_registers(io_number, 1, slave=self.slave)
            return result.registers[0]
        else:
            print("Invalid IO number. Use 0x0403 or 0x0404.")

    def set_io_parameters(self, io_param, value):
        if 0x0405 <= io_param <= 0x0410:
            self.client.write_register(io_param, value, slave=self.slave)
        else:
            print("Invalid IO parameter address. Use 0x0405 to 0x0410.")

    def read_io_parameters(self, io_param):
        if 0x0405 <= io_param <= 0x0410:
            result = self.client.read_holding_registers(io_param, 1, slave=self.slave)
            return result.registers[0]
        else:
            print("Invalid IO parameter address. Use 0x0405 to 0x0410.")

    def get_status(self):
        return self.gripper



# PGIGrippertest = PGIGripper()

# PGIGrippertest.connect("/dev/ttyUSB1", 1, 115200)
# PGIGrippertest.init_gripper()
# input()
# PGIGrippertest.set_position(0)

# print("gripper connected")

# # self.gripper.set_speed(100)
# # self.gripper.set_force(50)
# print(PGIGrippertest.read_position())