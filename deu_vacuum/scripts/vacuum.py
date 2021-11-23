#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Vacuum: # 메인 함수 호출 시 불려옴

    def __init__(self):
        self.wallfallow = wallFallow() # 벽 한 바퀴 도는 클래스
        self.startclean = startClean() # 청소 실행
        self.finished = False
        self.time = 0

    def vac_start(self):
        self.time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.finished is True:
                print('소요시간 : ', self.time - rospy.Time.now())
                print('청소 면적 : ')
                print('청소가 완료되었습니다, 종료합니다.')
                break
            else:
                if self.wallfallow.wallFinish is False: # 벽을 한 바퀴 돌기 전 실행된다.
                    self.wallfallow.following()
                    self.wallfallow.data()

                elif self.wallfallow.is_fin > 0 or self.wallfallow.wallFinish is True : # 벽을 한 바퀴 다 돈 후 실행된다.
                    self.startclean.set_angle()

class wallFallow: #벽타기 클래스
    is_fin = 0

    def __init__(self):
        # twist & Goal
        self.driving_forward = True # 직진하는지 체크한다.
        self.turn_isRight = False # true일 경우 오른쪽으로 회전, false일 경우 왼쪽으로 회전한다.
        self.savaData = False # 좌표 기억을 위해 타이밍 체크한다.
        self.rotate = False # true이면 회전한다.
        self.first_check = True # 벽을 돌기 위해 처음 회전한다.
        self.turn = Twist()
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.wallFinish = False # 벽 한바퀴 돌았는지 확인한다.

        # scan을 초기화한다.
        self.range_ahead = 1  # 정면
        self.range_left = 1  # 왼쪽
        self.range_right = 1  # 오른쪽

        # 터틀봇의 현재좌표을 초기화한다.
        self.odom_x = 0
        self.odom_y = 0

        # topic을 구독하고 발행한다.
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)  # odom
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.get_odom_data)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.roll = self.pitch = self.yaw = 0.0

        self.vc = startClean()

        self.dot_x = []
        self.dot_y = []

    def get_odom_data(self, msg): # 현재 터틀봇의 orientation 좌표를 받아서 roll,pitch,yaw 저장
        orientation_value = msg.pose.pose.orientation
        orientation_list = [orientation_value.x, orientation_value.y, orientation_value.z, orientation_value.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # 900 ahead
        self.range_left = msg.ranges[int(len(msg.ranges) / 1.3)]  # 1350 left
        self.range_right = msg.ranges[len(msg.ranges) / 4]  # 450 right


    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.xmap = self.odom_x + 11.0 # 내가 지정한 터틀봇 좌표 더하기
        self.ymap = self.odom_y + 9.0

    # 터틀봇이 회전할 경우 데이터를 저장한다.
    def data(self):
        if self.savaData is True:  # 터틀봇이 회전할 경우 실행된다.
            self.dot_x.append(self.xmap)
            self.dot_y.append(self.ymap)
            print('회전 좌표값 저장')
            self.savaData = False


    def following(self): # 벽 한 바퀴 돌 때 실행한다.
        # 처음 미로에 들어갔을 때 조건문을 실행한다.
        if self.first_check is True:
            # 90도 회전하여 미로에 진입한다.
            for i in range(10):  # 180도 회전한다.
                self.twist.angular.z = -math.degrees(1.5708)
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
                if math.degrees(self.yaw) == -90.00:
                    break
            first_move = Twist()
            first_move.linear.x = 0.5
            self.cmd_vel_pub.publish(first_move)
            rospy.sleep(5)
            self.first_check = False
        else:
            if ((abs(self.x) <= 0.4) and (abs(self.y) <= 0.4)) and self.driving_forward is False:
                self.wallFinish = True
                is_fin = 1
                print(is_fin)
                print('fin')
                self.vc.set_angle()
            else:
                if self.rotate is True:
                    for i in range(10):
                        self.cmd_vel_pub.publish(self.turn)
                        self.rate.sleep()
                    self.rotate = False
                else:
                    if self.driving_forward is True:
                        if self.range_ahead <= 0.4:  # 정면거리가 0.7이하이면 주행을 멈추고 회전한다.
                            self.driving_forward = False
                            print('회전 좌표값', self.xmap, self.ymap)
                        elif self.range_left > 1.0:  # 정면거리가 0.7이고 좌측이 뚫려있으면 정지한다.
                            self.driving_forward = False
                            self.turn_isRight = True
                            print('회전 좌표값', self.xmap, self.ymap)
                    else:
                        if self.range_ahead > 0.4:
                            self.driving_forward = True
                    twist = Twist()
                    # driving_forward = True 이면 1m/s로 직진한다.
                    if self.driving_forward is True:
                        twist.linear.x = 0.3
                        # 회전 시 range_left와 range_right 중 더 먼 값을 가진 쪽으로 회전을 수행한다.
                        if self.range_left < self.range_right:
                            self.turn.angular.z = -math.degrees(1.5708)
                    else: # 막히지 않은 벽을 돈다.
                        if self.turn_isRight is True:
                            self.savaData = True
                            self.driving_forward = True
                            self.turn_isRight = False
                            for i in range(4):
                                turn = Twist()
                                turn.angular.z = math.degrees(1.5708)
                                self.cmd_vel_pub.publish(turn)
                                rospy.sleep(1)
                                if i is 1:
                                    a = 3
                                else:
                                    a = 1
                                for i in range(a):
                                    gost = Twist()
                                    gost.linear.x = 0.5
                                    self.cmd_vel_pub.publish(gost)
                                    rospy.sleep(1)
                        else:
                            self.rotate = True
                            self.savaData = True
                    self.cmd_vel_pub.publish(twist)
                    self.rate.sleep()


class startClean: # 청소를 하기 위하여 실행한다.
    def __init__(self):
        self.driving_forward = True  # 직진하는지 확인한다.

        self.roll = self.pitch = self.yaw = 0.0

        self.turn_r = True
        self.x = self.y = 0.0

        # topic을 구독하고 발행한다.
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.get_odom_data)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.range_ahead = self.range_left = self.range_right = 0

        self.stack_x = []
        self.stack_x = map(float, self.stack_x)
        self.stack_y = []
        self.stack_y = map(float, self.stack_y)

        self.state_drive = True
        self.state_rotate = False
        self.flag_start = True

        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.count_point = 0

        self.cnt = 0
        self.angle = [-90, 90, 90, 90]

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # front
        self.range_left = msg.ranges[len(msg.ranges) / 12 * 3]  # left
        self.range_right = msg.ranges[len(msg.ranges) / 30 ]  # right

    # 지도로부터 로봇의 현재 좌표 및 각도를 저장한다.
    def get_odom_data(self, msg):
        orientation_value = msg.pose.pose.orientation
        orientation_list = [orientation_value.x, orientation_value.y, orientation_value.z, orientation_value.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def only_turn(self): # 돌면서 이동한다.
        for i in range(10):  # turn 180
            self.twist.angular.z = -math.degrees(1.5708)
            self.pub.publish(self.twist)
            rospy.sleep(1)

            if math.degrees(self.yaw) == -180.00:
                self.flag_start = False
                break
        for i in self.angle:
            while self.range_ahead >= 0.7 :
                self.twist.linear.x = 0.2
                self.pub.publish(self.twist)

            for j in range(10):  # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == self.angle[i]:
                    self.flag_start = False
                    break

            for i in range(10):  # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == 90.00:
                    self.flag_start = False
                    break
        self.set_angle()

    def set_angle(self): # 얘가 왔다리 갔다리
        if self.flag_start is True:
            for i in range(10): # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == -180.00:
                    self.flag_start = False
                    break

        else:
            if self.state_drive is True:
                if self.range_ahead <= 0.7:
                    if self.range_left <= 0.6 or self.range_right <= 0.6:
                        self.only_turn()
                    self.state_drive = False
            else:
                if self.range_ahead >= 0.7 :
                    self.state_drive = True
            twist = Twist()
            if self.state_drive is True:
                twist.linear.x = 0.2
            else:
                if (self.cnt % 2) == 0:
                    for i in range(2):
                        for j in range(2):
                            turning = Twist()
                            str = Twist()
                            turning.angular.z = math.degrees(1.5708)
                            self.pub.publish(turning)
                            rospy.sleep(1)
                        str.linear.x = 0.5
                        self.pub.publish(str)
                        rospy.sleep(1)
                    self.cnt = self.cnt + 1
                    print(self.cnt)
                    self.state_drive = True
                elif (self.cnt % 2) == 1:
                    for i in range(2):
                        for j in range(2):
                            turning = Twist()
                            str = Twist()
                            turning.angular.z = -math.degrees(1.5708)
                            self.pub.publish(turning)
                            rospy.sleep(1)
                        str.linear.x = 0.5
                        self.pub.publish(str)
                        rospy.sleep(1)
                    self.cnt = self.cnt + 1
                    print(self.cnt)
                    self.state_drive = True

# main
if __name__ == "__main__":
    rospy.init_node('vacuumClean')
    scan_move = Vacuum()
    scan_move.vac_start()
