#!/usr/bin/env python3
import rospy
import actionlib
import select
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import UInt16
from geometry_msgs.msg import PoseStamped, Twist, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # 문자열을 읽을 수 있는 함수

class patrol:
    def __init__(self):
        rospy.init_node('move_patrol')
        self.waypoints = [
            {'position': {'x': 5.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}},
            {'position': {'x': 5.8756658841920195, 'y': 1.8630986419405122}, 'orientation': {'z': 0.6858082829133141, 'w': 0.7277822470268781}},
            {'position': {'x': 1.8756658841920195, 'y': 0.9358025243813279}, 'orientation': {'z': 0.6888924506846787, 'w': 0.7248635674315943}},
            {'position': {'x': 0.630743275462133, 'y': -0.048679004629718914}, 'orientation': {'z': -0.9968777660395365, 'w': 0.07896024047597035}},
        ]
        self.get_location()
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty) # map clear
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.current_waypoint = 0
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
    
    def get_location(self): # parameter.yaml 안에 있는 좌표를 저장하는 함수
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

    def move_base_goal(self, waypoint): # goal 찍는 함수
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint['position']['x']
        goal.target_pose.pose.position.y = waypoint['position']['y']
        goal.target_pose.pose.orientation.z = waypoint['orientation']['z']
        goal.target_pose.pose.orientation.w = waypoint['orientation']['w']
        return goal
    

    def move(self):
        try:
            self.clear_costmaps_service()
            rospy.loginfo("성공적으로 Map을 clear 하였습니다")
        except rospy.ServiceException as e:
            rospy.logerr(f"실패: {e}")
            
        rate = rospy.Rate(1)  
        while not rospy.is_shutdown():
            waypoint = self.waypoints[self.current_waypoint] # 현재 waypoint 좌표 저장
            rospy.loginfo(f"Moving to waypoint : {self.current_waypoint + 1}번 좌표로 이동중입니다.")
            self.move_base_client.send_goal(self.move_base_goal(waypoint))
            while(not self.move_base_client.get_result()): # 키보드 입력 하면서 주행 컨트롤
                if isData():
                    c = sys.stdin.read(1)
                    if c == 'x':
                        self.client.cancel_all_goals()
                        rospy.loginfo("목적지를 종료합니다")
                        return stop
                    elif c == 'c':
                        self.client.cancel_all_goals()
                        rospy.loginfo("목적지를 변경합니다")
                    else:
                        rospy.loginfo("잘못 입력되었으니 다시 입력해주시기 바랍니다")

            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints) # 무한 반복 코드

            rate.sleep()
        

class stop():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def execute(self, userdata):
        self.client.cancel_all_goals()
    

    


if __name__ == '__main__':
    try:
        control = patrol()
        control.move()
    except rospy.ROSInterruptException:
        pass
