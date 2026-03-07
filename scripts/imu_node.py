#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import smbus
from sensor_msgs.msg import Temperature, Imu
from mpu_6050_driver.registers import *

class IMUNode(Node):

    def __init__(self):

        super().__init__('imu_node')

        self.bus = smbus.SMBus(1)
        self.ADDR = 0x68
        self.IMU_FRAME = "imu_link"

        self.bus.write_byte_data(self.ADDR, PWR_MGMT_1, 0)

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)

        self.timer = self.create_timer(0.02, self.publish_imu)
        self.temp_timer = self.create_timer(10.0, self.publish_temp)

    def read_word(self, adr):

        high = self.bus.read_byte_data(self.ADDR, adr)
        low = self.bus.read_byte_data(self.ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):

        val = self.read_word(adr)

        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def publish_temp(self):

        msg = Temperature()

        msg.temperature = self.read_word_2c(TEMP_H)/340.0 + 36.53
        msg.header.frame_id = self.IMU_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()

        self.temp_pub.publish(msg)

    def publish_imu(self):

        msg = Imu()

        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0

        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0

        msg.linear_acceleration.x = accel_x * 9.8
        msg.linear_acceleration.y = accel_y * 9.8
        msg.linear_acceleration.z = accel_z * 9.8

        msg.angular_velocity.x = gyro_x * 0.0174
        msg.angular_velocity.y = gyro_y * 0.0174
        msg.angular_velocity.z = gyro_z * 0.0174

        msg.header.frame_id = self.IMU_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()

        self.imu_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = IMUNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()