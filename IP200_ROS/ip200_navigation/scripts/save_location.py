#! /usr/bin/env python3
import rospy
import yaml
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from pynput import keyboard

class PoseSaver:
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pose)
        self.current_pose = PoseWithCovarianceStamped()
        rospy.loginfo('Start!')
        rospy.loginfo('Save: keyboard "s" button')

    def update_pose(self, data):
        self.current_pose = data.pose.pose

    def on_key_press(self, key):
        try:
            if key.char == 's':  # 's' key to save the current pose
                self.save_current_pose()
        except AttributeError:
            pass

    def save_current_pose(self):
        file_number = input("\n몇 번 좌표인지 숫자로 입력해주세요. (1~6): ")
        if 's' in file_number:
            file_number = file_number.replace('s', '')
            
        # aidl이 아니라 robo이면 /home/aidl을 /home/robo로 바꿔주세요.
        filename = os.path.join('/home/aidl/catkin_ws/src/IP200_ROS/ip200_teleop/config/', "params" + file_number.strip() + ".yaml")
        
        data = {
            "goal_position": {
                "x": self.current_pose.position.x,
                "y": self.current_pose.position.y,
            },
            "goal_orientation": {
                "z": self.current_pose.orientation.z,
                "w": self.current_pose.orientation.w
            }
        }
        
        with open(filename, 'w') as f:
            yaml.safe_dump(data, f)
        rospy.loginfo(f'Saved pose to params{file_number}')

if __name__ == '__main__':
    rospy.init_node('pose_saver_node')
    pose_saver = PoseSaver()

    # start the keyboard listener
    with keyboard.Listener(on_press=pose_saver.on_key_press) as listener:
        rospy.spin()
