#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
import math
from deu_maze.msg import LidarMeasure
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class addData:
    def __init__(self):
        self.data = list()

    def empty(self):
        if not self.data:
            return True
        else:
            return False

    def dataLength(self):
        return len(self.data)

    def push(self, tmpData):
        self.data.append(tmpData)

    def top(self):
        return self.data[len(self.data) - 1]


class inToMaze:  # 미로 탈출을 시작한다.
    def __init__(self):
        self.escapeMaze = EscapeMaze()
        self.retracemaze = BacktoMaze()

    def solve_start(self):
        while not rospy.is_shutdown():
            if (-8.55 < self.escapeMaze.map_x < -7.55) and (
                    5.05 < self.escapeMaze.map_y < 5.55) and self.escapeMaze.is_finised is True:
                break

            if ((7.75 < self.escapeMaze.map_x < 8.15) and (
                    -5.35 < self.escapeMaze.map_y < -4.90)) or self.escapeMaze.is_finised is True:
                self.escapeMaze.is_finised = True
                self.retracemaze.mazeBack()
            else:
                self.escapeMaze.escape()
                self.escapeMaze.saveData()


class EscapeMaze:
    stackData = addData()
    crossroadData = addData()

    def __init__(self):

        self.isGoState = True  # 주행 상태 확인한다.
        self.is_Rotate = False  # 회전 여부 확인한다.
        self.is_firstInMaze = True  # 터틀봇 미로 초기 진입 여부를 확인한다.
        self.is_first = True
        self.is_finised = False  # 도착 여부를 확인한다.
        self.wayCount = 1  # 갈 수 있는 길의 수를 확인한다.
        self.wayCheck = 0  # 갈림길 수를 계산한다.
        self.rotate = Twist()
        self.rate = rospy.Rate(10)

        # 터틀봇 방향
        self.turtlebot_direction = ['right', 'down', 'left', 'up']
        self.init_turtlebot_direction = 1  # 터틀봇 초기 방향 ('down')
        self.rotate_direction = 0  # 0은 반시계, 1은 시계 방향

        # 터틀봇 좌표(odom_data)
        self.turtlebot_x = 0
        self.turtlebot_y = 0

        # 맵 봐표(odom_data)
        self.map_x = 0
        self.map_y = 0

        self.lidar_sub = rospy.Subscriber('lidar_measure', LidarMeasure, self.lidar_callback)  # Lidar topic 구독한다.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)  # cmd_vel 발행한다.
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)  # odom 구독한다.

    def saveData(self):
        if self.is_Rotate:
            temp = [self.turtlebot_direction[self.init_turtlebot_direction], round(self.turtlebot_x, 3), round(self.turtlebot_y, 3)]
            # [ 방향, (전방, 후방, 좌측, 우측), 터틀봇 x 위치, 터틀봇 y 위치 ]

            if not self.stackData.empty():
                top = self.stackData.top()
                if (temp[1] == top[1]) and (temp[2] == top[2]):
                    self.stackData.data.pop()
                else:
                    self.stackData.push(temp)
            else:
                self.stackData.push(temp)

    def escape(self):
        if self.is_firstInMaze is True:
            for i in range(13):
                start_maze = Twist()
                start_maze.angular.z = -math.radians(90)  # 7.35 = math.radians(90)
                self.cmd_vel_pub.publish(start_maze)
                self.rate.sleep()
            self.is_firstInMaze = False

        else:
            twistData = Twist()
            if (-8.25 < self.map_x < -7.34) and (self.map_y > 4.38):  # 미로에 초기 진입할 경우
                twistData.linear.x = 1.0
                self.cmd_vel_pub.publish(twistData)
                self.rate.sleep()
                self.isGoState = True

            else:
                if self.is_Rotate is True:
                    if self.rotate_direction is 0:  # 반시계 방향으로 절대 방향 전환한다.
                        if (self.init_turtlebot_direction > 0):
                            self.init_turtlebot_direction -= 1
                        else:
                            self.init_turtlebot_direction = 3

                    elif self.rotate_direction is 1:  # 시계 방향으로 절대 방향 전환한다.
                        if (self.init_turtlebot_direction < 3):
                            self.init_turtlebot_direction += 1
                        else:
                            self.init_turtlebot_direction = 0

                    for i in range(10):
                        self.cmd_vel_pub.publish(self.rotate)
                        self.rate.sleep()
                    self.is_Rotate = False

                else:
                    if self.isGoState is True:
                        if self.range_ahead < 0.75:
                            self.isGoState = False

                        if self.wayCount > 2 and self.wayCheck is 0:
                            self.isGoState = False
                            self.wayCheck = 1

                        elif self.wayCount <= 2:
                            self.wayCheck = 0

                    else:
                        if self.range_ahead >= 0.75:
                            self.isGoState = True

                    if self.isGoState is True:
                        if self.wayCheck is 1:
                            for i in range(5):
                                tmpGo = Twist()
                                tmpGo.linear.x = 1.0
                                self.cmd_vel_pub.publish(tmpGo)
                                self.rate.sleep()
                        else:
                            twistData.linear.x = 1.0

                            if self.range_right >= 0.75:
                                self.rotate.angular.z = math.radians(90)
                                self.rotate_direction = 0  # 반시계 방향으로 회전한다.

                            elif self.range_left >= 0.75:
                                self.rotate.angular.z = -math.radians(90)
                                self.rotate_direction = 1  # 시계 방향으로 회전한다.
                    else:
                        if self.wayCount > 2 and self.wayCheck is 1:
                            for i in range(5):
                                tmpGo = Twist()
                                tmpGo.linear.x = 1.0
                                self.cmd_vel_pub.publish(tmpGo)
                                self.rate.sleep()

                        self.is_Rotate = True

                    self.cmd_vel_pub.publish(twistData)
                    self.rate.sleep()

    def lidar_callback(self, floatmsg):
        self.range_ahead = floatmsg.range_ahead
        self.range_left = floatmsg.range_left
        self.range_right = floatmsg.range_right
        self.range_rear = floatmsg.range_rear


    def odom_callback(self, msg):
        self.turtlebot_x = msg.pose.pose.position.x
        self.turtlebot_y = msg.pose.pose.position.y
        self.map_x = self.turtlebot_x - 7.92
        self.map_y = self.turtlebot_y + 5.30


class BacktoMaze:
    def __init__(self):
        self.m_solve = EscapeMaze()

        self.backToMaze = True  # 미로 도착 여부 확인한다.

        self.drivingFoward_back = True  # 주행 중인지 확인한다.
        self.rotateChecking_back = False  # 회전 중인지 확인한다.

        self.rate = rospy.Rate(10)
        self.keepData = self.m_solve.stackData.data  # 저장된 최단 경로

        self.turtlebot_current_direction = 3  # 터틀봇 초기 방향을 설정한다.

        self.lidar_sub = rospy.Subscriber('lidarMeasurement', LidarMeasure, self.m_solve.lidar_callback)  # Lidar topic 구독한다.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)  # cmd_vel 발행한다.
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.m_solve.odom_callback)  # odom 구독한다.

    def mazeBack(self):
        if self.backToMaze is True:
            for i in range(21):
                start_maze = Twist()
                start_maze.angular.z = -math.radians(90)  # 7.35 = math.radians(90)
                self.cmd_vel_pub.publish(start_maze)
                self.rate.sleep()
            self.backToMaze = False
        else:
            twistData = Twist()
            if len(self.keepData) is not 0:
                if (self.keepData[len(self.keepData) - 1][1] - 0.25 <= self.m_solve.turtlebot_x <=
                    self.keepData[len(self.keepData) - 1][1] + 0.25) and (
                        self.keepData[len(self.keepData) - 1][2] - 0.25 <= self.m_solve.turtlebot_y <=
                        self.keepData[len(self.keepData) - 1][2] + 0.25):
                    self.drivingFoward_back = False  # 주행을 정지한다.
                    self.rotateChecking_back = True  # 회전을 수행한다.

                    self.turtlebot_direction = self.m_solve.turtlebot_direction.index(
                        self.keepData[len(self.keepData) - 1][0])  # 방향을 설정한다.
                    self.turtlebot_direction = (self.turtlebot_direction + 2) % 4

                    if self.rotateChecking_back is True:
                        if self.drivingFoward_back is False:
                            turtle_current_left = self.turtlebot_current_direction - 1 if (
                                        self.turtlebot_current_direction > 0) else 3
                            turtle_current_right = self.turtlebot_current_direction + 1 if (
                                        self.turtlebot_current_direction < 3) else 0

                            rotateData = Twist()
                            if self.turtlebot_direction == turtle_current_left:
                                rotateData.angular.z = math.radians(90)
                                self.turtlebot_current_direction = turtle_current_left

                            elif self.turtlebot_direction == turtle_current_right:
                                rotateData.angular.z = math.radians(-90)
                                self.turtlebot_current_direction = turtle_current_right

                            for i in range(10):
                                self.cmd_vel_pub.publish(rotateData)
                                self.rate.sleep()

                            self.keepData.pop()
                            self.rotateChecking_back = False
                            self.drivingFoward_back = True

                else:
                    if self.rotateChecking_back is False:
                        if self.drivingFoward_back is True:
                            twistData.linear.x = 1.0

                self.cmd_vel_pub.publish(twistData)
                self.rate.sleep()

            else:
                twistData.linear.x = 1.0
                self.cmd_vel_pub.publish(twistData)
                self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node('solveMaze')
    maze = inToMaze()
    maze.solve_start()



