#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from smach import State,StateMachine
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State,StateMachine
import subprocess
from time import sleep
import sys
import select
import tty
import termios
import smach_ros
import os

waypoints = [
    ['A', (2.64499974251, 0.509999752045), (0.707106796641, 0.707106765732)],
    ['B', (2.47258925438, 4.40223407745), (0.999878011069, -0.0156193143396)],
    ['C', (-1.40137755871, 4.37288618088), (-0.703452686599, 0.71074208945)],
    ['D', (-1.32800757885, 0.0733705535531), (0.0142813322057, 0.999898016575)]
]
# waypoints = [
#             ['A', {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}}],
#             ['B', {'position': {'x': 1.8756658841920195, 'y': 1.8630986419405122}, 'orientation': {'z': 0.6858082829133141, 'w': 0.7277822470268781}}],
#             ['C', {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}}],
#             ['D', {'position': {'x': 0.630743275462133, 'y': -0.048679004629718914}, 'orientation': {'z': -0.9968777660395365, 'w': 0.07896024047597035}}]
#         ]

# waypoints = [
#             {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}},
#             {'position': {'x': 1.8756658841920195, 'y': 1.8630986419405122}, 'orientation': {'z': 0.6858082829133141, 'w': 0.7277822470268781}},
#             {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}},
#             {'position': {'x': 0.630743275462133, 'y': -0.048679004629718914}, 'orientation': {'z': -0.9968777660395365, 'w': 0.07896024047597035}},
#             {'position': {'x': -0.08097582839788423, 'y': -0.04931709957737404}, 'orientation': {'z': -0.999884438319063, 'w': 0.015202302699649282}},
#             {'position': {'x': 0.630743275462133, 'y': -0.048679004629718914}, 'orientation': {'z': -0.9968777660395365, 'w': 0.07896024047597035}},
#         ]



def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # 문자열을 읽을 수 있는 함수

class A(State): # one의 지점으로 가는 상태를 나타내는 구문
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop'])
         # Get an action client
        self.get_location()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() # action client을 얻는 것 movebaseaction
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def get_location(self):
        for i in range(4):
            if i==0:
                self.waypoints[i]['position']['x'] = rospy.get_param('~params0/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('~params0/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('~params0/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('~params0/goal_orientation/w')
            elif i==1:
                self.waypoints[i]['position']['x'] = rospy.get_param('~params1/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('~params1/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('~params1/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('~params1/goal_orientation/w')
            elif i==2:
                self.waypoints[i]['position']['x'] = rospy.get_param('~params2/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('~params2/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('~params2/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('~params2/goal_orientation/w')
            elif i==3:
                self.waypoints[i]['position']['x'] = rospy.get_param('~params3/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('~params3/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('~params3/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('~params3/goal_orientation/w')
    
    def execute(self, userdata): # 실행하는 함수 execute
        print('A')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 'x':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop'
        self.client.wait_for_result() # goal을 보내고나서 s키를 누르면 골을 취소하고 stop으로 보낸뒤 기다린다.
        print('success')
        return 'success'
    
class B(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop'])
         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        print('B')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 'x':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop'
        print(self.client.get_result())
        self.client.wait_for_result()
        print('success')
        return 'success'

class C(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop'])
         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

        
    def execute(self, userdata):
        print('C')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 'x':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop'
        print(self.client.get_result())
        self.client.wait_for_result()
        print('success')
        return 'success'

class D(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop'])
         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        print('D')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 'x':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop'
        print(self.client.get_result())
        self.client.wait_for_result()
        print('success')
        return 'success'

class stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['A', 'B', 'C', 'D'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def execute(self, userdata):
        self.client.cancel_all_goals()
        while True:
            if isData():
                c = sys.stdin.read(1)
                if c != 'x':
                    break # 'x'를 누르면 break 실행
                print('resume')
                return 'success' # success로 보낸다

if __name__ == '__main__':
    rospy.init_node('patrol') # 노드 형성
    patrol = StateMachine(outcomes=['success']) # StateMchine 상태 변환
    with patrol:
        StateMachine.add('A', A(waypoints[0][1], waypoints[0][2]), transitions={'success': 'B', 'stop': 'stop'})  # success를 하면 하나의 지점으로 이동하고 stop 하면 정지하는 변환 수행
        StateMachine.add('B', B(waypoints[1][1], waypoints[1][2]), transitions={'success': 'C', 'stop': 'stop'})
        StateMachine.add('C', C(waypoints[2][1], waypoints[2][2]), transitions={'success': 'D', 'stop': 'stop'})
        StateMachine.add('D', D(waypoints[3][1], waypoints[3][2]), transitions={'success': 'A', 'stop': 'stop'})
        StateMachine.add('stop', stop(), transitions={'A': 'A', 'B': 'B', 'C': 'C', 'D': 'D'})
    
    outcome = patrol.execute() # 실행
    rospy.spin() # spin으로 여기까지 계속 실행