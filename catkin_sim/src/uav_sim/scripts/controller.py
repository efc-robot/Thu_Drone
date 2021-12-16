#!/usr/bin/python
#-*- encoding: utf8 -*-

# 对windows.world的一个简单控制策略
# 结合tello的控制接口，控制无人机从指定位置起飞，识别模拟火情标记（红色），穿过其下方对应的窗户，并在指定位置降落
# 本策略尽量使无人机的偏航角保持在初始值（90度）左右
# 运行roslaunch uav_sim windows.launch后，再在另一个终端中运行rostopic pub /tello/cmd_start std_msgs/Bool "data: 1"即可开始飞行
# 代码中的decision()函数和switchNavigatingState()函数共有3个空缺之处，需要同学们自行补全（每个空缺之处需要填上不超过3行代码）

from scipy.spatial.transform import Rotation as R
from collections import deque
from enum import Enum
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ControllerNode:
    class FlightState(Enum):  # 飞行状态
        WAITING = 1
        NAVIGATING = 2
        DETECTING_TARGET = 3
        LANDING = 4

    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.logwarn('Controller node set up.')

        # 无人机在世界坐标系下的位姿
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype=np.float64)

        self.image_ = None
        self.color_range_ = [(0, 43, 46), (6, 255, 255)] # 红色的HSV范围
        self.bridge_ = CvBridge()

        self.flight_state_ = self.FlightState.WAITING
        self.navigating_queue_ = deque()  # 存放多段导航信息的队列，队列元素为二元list，list的第一个元素代表导航维度（'x' or 'y' or 'z'），第二个元素代表导航目的地在该维度的坐标
        self.navigating_dimension_ = None  # 'x' or 'y' or 'z'
        self.navigating_destination_ = None
        self.next_state_ = None  # 完成多段导航后将切换的飞行状态

        self.window_x_list_ = [1.75, 4.25, 6.75] # 窗户中心点对应的x值

        self.is_begin_ = False

        self.commandPub_ = rospy.Publisher('/tello/cmd_string', String, queue_size=100)  # 发布tello格式控制信号

        self.poseSub_ = rospy.Subscriber('/tello/states', PoseStamped, self.poseCallback)  # 接收处理含噪无人机位姿信息
        self.imageSub_ = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.imageCallback)  # 接收摄像头图像
        self.imageSub_ = rospy.Subscriber('/tello/cmd_start', Bool, self.startcommandCallback)  # 接收开始飞行的命令

        rate = rospy.Rate(0.3)
        while not rospy.is_shutdown():
            if self.is_begin_:
                self.decision()
            rate.sleep()
        rospy.logwarn('Controller node shut down.')

    # 按照一定频率进行决策，并发布tello格式控制信号
    def decision(self):
        if self.flight_state_ == self.FlightState.WAITING:  # 起飞并飞至离墙体（y = 3.0m）适当距离的位置
            rospy.logwarn('State: WAITING')
            self.publishCommand('takeoff')
            self.navigating_queue_ = deque([['y', 1.8]])
            self.switchNavigatingState()
            self.next_state_ = self.FlightState.DETECTING_TARGET

        elif self.flight_state_ == self.FlightState.NAVIGATING:
            rospy.logwarn('State: NAVIGATING')
            # 如果yaw与90度相差超过正负10度，需要进行旋转调整yaw
            (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
            yaw_diff = yaw - 90 if yaw > -90 else yaw + 270
            if yaw_diff > 10:  # clockwise
                self.publishCommand('cw %d' % (int(yaw_diff) if yaw_diff > 15 else 15))
                return
            elif yaw_diff < -10:  # counterclockwise
                # TODO 1: 发布相应的tello控制命令
                self.publishCommand('ccw %d' % (int(-yaw_diff) if yaw_diff < -15 else 15))
                return
                # end of TODO 1

            dim_index = 0 if self.navigating_dimension_ == 'x' else (1 if self.navigating_dimension_ == 'y' else 2)
            dist = self.navigating_destination_ - self.t_wu_[dim_index]
            if abs(dist) < 0.3:  # 当前段导航结束
                self.switchNavigatingState()
            else:
                dir_index = 0 if dist > 0 else 1  # direction index
                # TODO 2: 根据维度（dim_index）和导航方向（dir_index）决定使用哪个命令
                command_matrix = [['right ', 'left '], ['forward ', 'back '], ['up ', 'down ']]
                command = command_matrix[dim_index][dir_index]
                # end of TODO 2
                if abs(dist) > 1.5:
                    self.publishCommand(command+'100')
                else:
                    self.publishCommand(command+str(int(abs(100*dist))))

        elif self.flight_state_ == self.FlightState.DETECTING_TARGET:
            rospy.logwarn('State: DETECTING_TARGET')
            # 如果无人机飞行高度与标识高度（1.75m）相差太多，则需要进行调整
            if self.t_wu_[2] > 2.0:
                self.publishCommand('down %d' % int(100*(self.t_wu_[2] - 1.75)))
                return
            elif self.t_wu_[2] < 1.5:
                self.publishCommand('up %d' % int(-100*(self.t_wu_[2] - 1.75)))
                return
            # 如果yaw与90度相差超过正负10度，需要进行旋转调整yaw
            (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
            yaw_diff = yaw - 90 if yaw > -90 else yaw + 270
            if yaw_diff > 10:  # clockwise
                self.publishCommand('cw %d' % (int(yaw_diff) if yaw_diff > 15 else 15))
                return
            elif yaw_diff < -10:  # counterclockwise
                self.publishCommand('ccw %d' % (int(-yaw_diff) if yaw_diff < -15 else 15))
                return

            if self.detectTarget():
                #rospy.loginfo('Target detected.')
                # 根据无人机当前x坐标和偏航角判断正确的窗口是哪一个
                # 实际上可以结合目标在图像中的位置和相机内外参数得到标记点较准确的坐标，这需要相机成像的相关知识
                # 此处仅仅是做了一个粗糙的估计
                sign_x = self.t_wu_[0]-1.5*math.sin((yaw-90)*math.pi/180)  # 估计着火点x坐标
                win_dist = [abs(sign_x-win_x) for win_x in self.window_x_list_]
                win_index = win_dist.index(min(win_dist))  # 正确的窗户编号
                self.navigating_queue_ = deque([['y', 2.4], ['z', 1.0], ['x', self.window_x_list_[win_index]], ['y', 5.0], ['x', 7.0]])  # 通过窗户并导航至终点上方
                self.switchNavigatingState()
                self.next_state_ = self.FlightState.LANDING
            else:
                if self.t_wu_[0] > 7.5:
                    rospy.loginfo('Detection failed, ready to land.')
                    self.flight_state_ = self.FlightState.LANDING
                else:  # 向右侧平移一段距离，继续检测
                    self.publishCommand('right 75')

        elif self.flight_state_ == self.FlightState.LANDING:
            rospy.logwarn('State: LANDING')
            self.publishCommand('land')
        else:
            pass

    # 在向目标点导航过程中，更新导航状态和信息
    def switchNavigatingState(self):
        if len(self.navigating_queue_) == 0:
            self.flight_state_ = self.next_state_
        else: # 从队列头部取出无人机下一次导航的状态信息
            next_nav = self.navigating_queue_.popleft()
            # TODO 3: 更新导航信息和飞行状态
            self.navigating_dimension_ = next_nav[0]
            self.navigating_destination_ = next_nav[1]
            self.flight_state_ = self.FlightState.NAVIGATING
            # end of TODO 3

    # 判断是否检测到目标
    def detectTarget(self):
        if self.image_ is None:
            return False
        image_copy = self.image_.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, self.color_range_[0], self.color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False

    # 向相关topic发布tello命令
    def publishCommand(self, command_str):
        msg = String()
        msg.data = command_str
        self.commandPub_.publish(msg)

    # 接收无人机位姿
    def poseCallback(self, msg):
        self.t_wu_ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.R_wu_ = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        pass

    # 接收相机图像
    def imageCallback(self, msg):
        try:
            self.image_ = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            print(err)

    # 接收开始信号
    def startcommandCallback(self, msg):
        self.is_begin_ = msg.data


if __name__ == '__main__':
    cn = ControllerNode()

