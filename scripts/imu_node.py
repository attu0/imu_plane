#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import smbus
import time
import math
from sensor_msgs.msg import Temperature, Imu
from mpu_6050_driver.registers import *

class IMUNode(Node):

    def __init__(self):

        super().__init__('imu_node')

        self.bus = smbus.SMBus(1)
        self.ADDR = 0x68
        self.IMU_FRAME = "imu_link"

        # Allow sensor to stabilize
        time.sleep(0.5)

        # Retry initialization
        for i in range(5):
            try:
                self.bus.write_byte_data(self.ADDR, PWR_MGMT_1, 0)
                self.get_logger().info("MPU6050 initialized successfully")
                break
            except Exception as e:
                self.get_logger().warn(f"I2C init failed, retrying... {e}")
                time.sleep(0.2)

        time.sleep(0.1)

        # Yaw tracking (gyro integration)
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.publish_imu)   # 20 Hz
        self.temp_timer = self.create_timer(10.0, self.publish_temp)

    def read_word(self, adr):
        try:
            high = self.bus.read_byte_data(self.ADDR, adr)
            low = self.bus.read_byte_data(self.ADDR, adr + 1)
            return (high << 8) + low
        except Exception as e:
            self.get_logger().warn(f"I2C read error: {e}")
            return 0

    def read_word_2c(self, adr):

        val = self.read_word(adr)

        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def publish_temp(self):

        msg = Temperature()

        temp_raw = self.read_word_2c(TEMP_H)
        msg.temperature = temp_raw / 340.0 + 36.53

        msg.header.frame_id = self.IMU_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()

        self.temp_pub.publish(msg)

    def publish_imu(self):

        msg = Imu()

        # Time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Read accelerometer
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0

        # Read gyroscope (deg/s → rad/s)
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0 * 0.0174
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0 * 0.0174
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0 * 0.0174

        # Convert acceleration to m/s²
        msg.linear_acceleration.x = accel_x * 9.8
        msg.linear_acceleration.y = accel_y * 9.8
        msg.linear_acceleration.z = accel_z * 9.8

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        # ==========================
        # ORIENTATION
        # ==========================

        # Roll & Pitch from accelerometer
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # Yaw from gyro integration
        self.yaw += gyro_z * dt
        yaw = self.yaw

        # Quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        msg.header.frame_id = self.IMU_FRAME
        msg.header.stamp = current_time.to_msg()

        self.imu_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = IMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()