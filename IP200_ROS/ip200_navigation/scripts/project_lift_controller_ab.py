#!/usr/bin/env python3
import rospy
import actionlib
import dynamic_reconfigure.client

from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16
from geometry_msgs.msg import PoseStamped, Twist, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class FactoryControl:
# A, B, C, D 지점의 좌표와 회전을 저장합니다.
    def __init__(self):
        rospy.init_node('move_robot_sequence')
        self.waypoints = [
            {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}},
            {'position': {'x': 1.8756658841920195, 'y': 1.8630986419405122}, 'orientation': {'z': 0.6858082829133141, 'w': 0.7277822470268781}},
            {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}},
            {'position': {'x': 0.630743275462133, 'y': -0.048679004629718914}, 'orientation': {'z': -0.9968777660395365, 'w': 0.07896024047597035}},
            {'position': {'x': -0.08097582839788423, 'y': -0.04931709957737404}, 'orientation': {'z': -0.999884438319063, 'w': 0.015202302699649282}},
            {'position': {'x': 0.630743275462133, 'y': -0.048679004629718914}, 'orientation': {'z': -0.9968777660395365, 'w': 0.07896024047597035}},
        ]
        self.get_location()

        

    def get_location(self):
        for i in range(6):
            if i==0 or i==2:
                self.waypoints[i]['position']['x'] = rospy.get_param('project_lift_controller_ab/params1/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('project_lift_controller_ab/params1/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('project_lift_controller_ab/params1/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('project_lift_controller_ab/params1/goal_orientation/w')
            elif i==1:
                self.waypoints[i]['position']['x'] = rospy.get_param('project_lift_controller_ab/params2/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('project_lift_controller_ab/params2/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('project_lift_controller_ab/params2/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('project_lift_controller_ab/params2/goal_orientation/w')
            elif i==3 or i==5:
                self.waypoints[i]['position']['x'] = rospy.get_param('project_lift_controller_ab/params3/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('project_lift_controller_ab/params3/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('project_lift_controller_ab/params3/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('project_lift_controller_ab/params3/goal_orientation/w')
            elif i==4:
                self.waypoints[i]['position']['x'] = rospy.get_param('project_lift_controller_ab/params4/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('project_lift_controller_ab/params4/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('project_lift_controller_ab/params4/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('project_lift_controller_ab/params4/goal_orientation/w')

    def move_base_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint['position']['x']
        goal.target_pose.pose.position.y = waypoint['position']['y']
        goal.target_pose.pose.orientation.z = waypoint['orientation']['z']
        goal.target_pose.pose.orientation.w = waypoint['orientation']['w']
        return goal

    def move_cmd_velocity(self, velocity):
        move_cmd = Twist()
        move_cmd.linear = Vector3(velocity, 0, 0)
        move_cmd.angular = Vector3(0, 0, 0)
        return move_cmd

    def change_inflation_radius(self, inflation_radius):
        self.inflation_client.update_configuration({'inflation_radius': inflation_radius})
        rospy.loginfo(f'inflation_radius: {inflation_radius}')
        rospy.sleep(0.25)

    def change_min_vel_x(self, min_vel_x):
        self.dyn_reconf_client.update_configuration({'min_vel_x': min_vel_x})
        rospy.loginfo(f'min_vel_x: {min_vel_x}')
        rospy.sleep(0.25)
    
    def change_xy_tolerance(self, xy_goal_tolerance):
        self.dyn_reconf_client.update_configuration({'xy_goal_tolerance': xy_goal_tolerance})
        rospy.loginfo(f'xy_goal_tolerance: {xy_goal_tolerance}')
        rospy.sleep(0.25)

    def change_yaw_tolerance(self, yaw_goal_tolerance):
        self.dyn_reconf_client.update_configuration({'yaw_goal_tolerance': yaw_goal_tolerance})
        rospy.loginfo(f'yaw_goal_tolerance: {yaw_goal_tolerance}')
        rospy.sleep(0.25)

    def lift_control(self, command):
        # (0: Up / 1: Down / 2: Stop)
        lift = None
        if command == "up":
            lift = 0
            rospy.loginfo("Lift Up")
            # lift_pub.publish(lift)  # (0: Up / 1: Down / 2: Stop)
            now = rospy.get_rostime()
            while (rospy.get_rostime() - now).to_sec() < float(self.lift_time):
                pass

        elif command == "down":
            lift = 1
            rospy.loginfo("Lift down")
            # lift_pub.publish(lift)  # (0: Up / 1: Down / 2: Stop)
            now = rospy.get_rostime()
            while (rospy.get_rostime() - now).to_sec() < float(self.lift_time):
                pass
    
    def joy_callback(self, data):
        # triangle = data.buttons[2]
        # rectangular = data.buttons[3]
        current_button_state = data.buttons[2]
        rospy.loginfo(f'{current_button_state}')
        if self.last_button_state==1 and current_button_state==0 and (not self.moving):
        # if data.buttons[2] == 1 and (not self.moving):
            self.moving = True
            rospy.loginfo('joystick pushed!')

            self.move_robot_sequence('a')
            self.move_robot_sequence('b')
            
            # if not self.arrived_a:
            #     self.move_robot_sequence('a')

            #     self.arrived_a = True
            #     self.arrived_b = False

            # elif not self.arrived_b:
            #     self.move_robot_sequence('b')

            #     self.arrived_a = False
            #     self.arrived_b = True

        self.last_button_state = current_button_state

    def move_robot_sequence(self, point):
        try:
            self.clear_costmaps_service()
            rospy.loginfo("Successfully clear costmap.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
        for i, waypoint in enumerate(self.waypoints):
            if point == 'a':
                if i==3 or i==4 or i==5:
                    continue
            elif point == 'b':
                if i==0 or i==1 or i==2:
                    continue

            goal = self.move_base_goal(waypoint)
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"목적지에 도착: {waypoint}")

                if i == 0:
                    rospy.loginfo("A2 지점에 도착")
                    self.change_inflation_radius(0.1)
                    self.lift_control("down")

                elif i == 1:
                    rospy.loginfo("A1 지점에 도착")
                    self.change_inflation_radius(0.4)
                    self.change_min_vel_x(-0.15)
                    self.change_xy_tolerance(0.13)  # 0.1m
                    self.change_yaw_tolerance(0.2) 
                    self.lift_control("up")

                elif i == 2:
                    rospy.loginfo("A2 지점에 도착")
                    self.change_inflation_radius(self.default_inflation_radius)  # 0.75
                    self.change_min_vel_x(self.default_min_vel_x)  # 0.0 m/s
                    self.change_xy_tolerance(self.default_xy_goal_tolerance)
                    self.change_yaw_tolerance(self.default_yaw_goal_tolerance)
                    self.lift_control("down")
                    self.moving = False

                elif i == 3:
                    rospy.loginfo("B2 지점에 도착")
                    self.change_inflation_radius(0.1)
                    self.lift_control("up")
                
                elif i == 4:
                    rospy.loginfo("B1 지점에 도착")
                    self.change_inflation_radius(0.4)
                    self.change_min_vel_x(-0.15)
                    self.change_xy_tolerance(0.13)  # 0.1m
                    self.change_yaw_tolerance(0.2) 
                    self.lift_control("down")
                
                elif i == 5:
                    rospy.loginfo("B2 지점에 도착")
                    self.change_inflation_radius(self.default_inflation_radius)  # 0.75
                    self.change_min_vel_x(self.default_min_vel_x)  # 0.0 m/s
                    self.change_xy_tolerance(self.default_xy_goal_tolerance)
                    self.change_yaw_tolerance(self.default_yaw_goal_tolerance)
                    self.moving = False

                try:
                    self.clear_costmaps_service()
                    rospy.loginfo("Successfully clear costmap.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
            else:
                rospy.logerr(f"이동 실패: {waypoint}")


if __name__ == '__main__':
    try:
        fc = FactoryControl()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass