#!/usr/bin/env python3
# HFI-A9 IMU ROS2 node (adapted from handsfree_ros_imu)
# Protocol: AA 55, baudrate 921600
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import struct
import math
import time


def checkSum(list_data, check_data):
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])


def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str = (hex(raw_data[i]   | 0xff00)[4:6] +
                    hex(raw_data[i+1] | 0xff00)[4:6] +
                    hex(raw_data[i+2] | 0xff00)[4:6] +
                    hex(raw_data[i+3] | 0xff00)[4:6])
        ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class HFIImuNode(Node):
    def __init__(self):
        super().__init__('hfi_imu')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        self.buff = {}
        self.key = 0
        self.angle_degree = [0.0, 0.0, 0.0]
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]

        # Kept so the port can be reopened after a USB re-enumeration.
        self.port = port
        self.baud = baudrate
        self.ser = None
        self._last_open_attempt = 0.0

        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
            # The CP2102-connected IMU starts streaming only after DTR/RTS are asserted.
            self.ser.dtr = True
            self.ser.rts = True
            time.sleep(2.0)
            self.get_logger().info(f'Opened {port} at {baudrate} baud')
        except Exception as e:
            # Not fatal any more: the device may simply not be up yet, and the
            # read timer retries.  Dying here would mean a node that cannot
            # survive being started a second too early.
            self.get_logger().error(f'Failed to open serial: {e}; will retry')
            self.ser = None

        self.timer = self.create_timer(0.002, self.read_serial)

    def handle_byte(self, raw_data):
        self.buff[self.key] = raw_data
        self.key += 1

        if self.buff[0] != 0xaa:
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:
            return

        data_buff = list(self.buff.values())

        if self.buff[2] == 0x2c:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                data = hex_to_ieee(data_buff[7:47])
                self.angularVelocity = data[1:4]
                self.acceleration = data[4:7]
                self.magnetometer = data[7:10]
            else:
                self.get_logger().warn('CRC fail: 0x2c')
            # This frame has no fresh attitude angle.  Publishing it would repeat
            # the previous pitch with a new timestamp and create a false derivative
            # spike in the balance controller.
            self.buff = {}
            self.key = 0
            return

        elif self.buff[2] == 0x14:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                self.angle_degree = data[1:4]
            else:
                self.get_logger().warn('CRC fail: 0x14')
                self.buff = {}
                self.key = 0
                return

        else:
            self.buff = {}
            self.key = 0
            return

        self.buff = {}
        self.key = 0

        # Only a 0x14 frame reaches here, so every published orientation is new.
        now = self.get_clock().now().to_msg()

        # publish Imu
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'

        roll  =  self.angle_degree[0] * math.pi / 180
        pitch = -self.angle_degree[1] * math.pi / 180
        yaw   = -self.angle_degree[2] * math.pi / 180
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        imu_msg.angular_velocity.x = self.angularVelocity[0]
        imu_msg.angular_velocity.y = self.angularVelocity[1]
        imu_msg.angular_velocity.z = self.angularVelocity[2]

        acc_k = math.sqrt(sum(a ** 2 for a in self.acceleration)) or 1.0
        imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
        imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
        imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k

        self.imu_pub.publish(imu_msg)

        # publish MagneticField
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = 'imu_link'
        mag_msg.magnetic_field.x = self.magnetometer[0]
        mag_msg.magnetic_field.y = self.magnetometer[1]
        mag_msg.magnetic_field.z = self.magnetometer[2]
        self.mag_pub.publish(mag_msg)

    def read_serial(self):
        if self.ser is None and not self._reopen():
            return
        try:
            count = self.ser.in_waiting
            if count > 0:
                data = self.ser.read(count)
                for byte in data:
                    self.handle_byte(byte)
        except Exception as e:
            # A USB serial device that re-enumerates leaves this node holding a
            # dead file descriptor.  Reading it returns Errno 5 forever, and
            # without reopening the node stays alive publishing nothing while
            # the last orientation it managed to parse sits frozen on the
            # topic.  Downstream that is worse than silence: the balance
            # controller saw a constant pitch with zero rate, concluded the
            # robot was standing still and upright, and engaged.  Observed
            # after a boot where the adapter came back as ttyUSB1.
            self.get_logger().error(f'Serial error: {e}; closing and retrying')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.key = 0            # drop any half-parsed frame

    def _reopen(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_open_attempt < 1.0:
            return False
        self._last_open_attempt = now
        try:
            # Reopen by the by-id path, which follows the device across
            # re-enumeration; the ttyUSB number does not.
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.5)
            self.ser.dtr = True
            self.ser.rts = True
            self.get_logger().info(f'reopened {self.port}')
            return True
        except Exception as e:
            self.get_logger().warn(f'reopen failed: {e}')
            self.ser = None
            return False


def main():
    rclpy.init()
    node = HFIImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
