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

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.dyn_reconf_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS', timeout=30)
        self.inflation_client = dynamic_reconfigure.client.Client('move_base/global_costmap/inflation_layer', timeout=30)
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        
        self.moving = False
        self.arrived_a = False
        self.arrived_b = True

        self.lift_time = rospy.get_param('~lift_time', 5.0)
        # self.default_max_vel_x = rospy.get_param('move_base/DWAPlannerROS/max_vel_x')
        self.default_min_vel_x = rospy.get_param('move_base/DWAPlannerROS/min_vel_x')
        self.default_inflation_radius = rospy.get_param('move_base/global_costmap/inflation_layer/inflation_radius')
        self.default_xy_goal_tolerance = rospy.get_param('move_base/DWAPlannerROS/xy_goal_tolerance')
        self.default_yaw_goal_tolerance = rospy.get_param('move_base/DWAPlannerROS/yaw_goal_tolerance')

        self.last_button_state = 0

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

                elif i == 1:
                    rospy.loginfo("A1 지점에 도착")
                    self.lift_control("up")

                elif i == 2:
                    rospy.loginfo("A2 지점에 도착")
                    self.moving = False

                elif i == 3:
                    rospy.loginfo("B2 지점에 도착")
                    self.change_inflation_radius(0.1)
                    self.lift_control("up")
                
                elif i == 4:
                    rospy.loginfo("B1 지점에 도착")
                    self.lift_control("down")
                
                elif i == 5:
                    rospy.loginfo("B2 지점에 도착")
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