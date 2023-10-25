#!/usr/bin/env python3

import rospy
import serial
from sensor_msgs.msg import Imu

class IMUPublisher:
    def __init__(self):
        rospy.init_node('imu_publisher', anonymous=True)
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.imu_msg = Imu()
        
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    def run(self):
        while not rospy.is_shutdown():
            data = self.ser.readline().decode().strip()
            if data:
                ax, ay, az, gx, gy, gz  = data.split(',')
                ax, ay, az = float(ax), float(ay), float(az)
                gx, gy, gz = float(gx), float(gy), float(gz)
                self.imu_msg.header.stamp = rospy.Time.now()
                self.imu_msg.header.frame_id = "imu"
                self.imu_msg.linear_acceleration.x = ax / 16384.0
                self.imu_msg.linear_acceleration.y = ay / 16384.0
                self.imu_msg.linear_acceleration.z = az / 16384.0
                self.imu_msg.angular_velocity.x = gx / 131.0
                self.imu_msg.angular_velocity.y = gy / 131.0
                self.imu_msg.angular_velocity.z = gz / 131.0
                self.imu_pub.publish(self.imu_msg)

if __name__ == '__main__':
    imu_publisher = IMUPublisher()
    imu_publisher.run()