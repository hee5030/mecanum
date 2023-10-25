#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class GetYFromJoy:
    def __init__(self):
        rospy.init_node('get_y_from_joy')

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

        self.lin_x_scale = 0.1
        self.lin_y_scale = 0.1
        self.ang_z_scale = 0.3
        
        self.button_clicked = 0
        self.last_button_clicked = 0

    def joy_callback(self, data):
        # 왼쪽 스틱 좌우 -> data.axes[0]
        # 왼쪽 스틱 상하 -> data.axes[1]
        # 오른쪽 스틱 좌우 -> data.axes[3]
        # 오른쪽 스틱 상하 -> data.axes[4]
        # o 버튼 -> data.buttons[1]

        self.button_clicked = data.buttons[1]
        if self.button_clicked == 0 and self.last_button_clicked==1:
            vel = Twist()
            self.cmd_vel_pub.publish(vel)

        if data.buttons[1] == 1:
            if data.axes[0]==0.0 and data.axes[1]==0.0 and data.axes[3]==0.0 and data.axes[4]==0.0:
                vel = Twist()
                self.cmd_vel_pub.publish(vel)

            else:
                vel = Twist()
                vel.linear.x = data.axes[1] * self.lin_x_scale
                vel.linear.y = data.axes[0] * self.lin_y_scale
                vel.angular.z = data.axes[3] * self.ang_z_scale
                self.cmd_vel_pub.publish(vel)

        self.last_button_clicked = self.button_clicked
        
if __name__ == '__main__':
    try:
        pn = GetYFromJoy()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass