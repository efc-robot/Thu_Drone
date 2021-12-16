#!/usr/bin/python
#-*- encoding: utf8 -*-

from tools import pressAnyKeyExit, getPackagePath, rostime2str
from scipy.spatial.transform import Rotation as R
from enum import Enum
import rospy
import yaml
import numpy as np
import os
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

class JudgeNode:
    class NodeState(Enum):
        WAITING = 1
        FLYING = 2
        LANDING = 3
        LANDED = 4

    def __init__(self):
        rospy.init_node('judge_node', anonymous=True)

        self.ros_path_ = getPackagePath('uav_sim')
        self.readYaml()

        # 无人机在世界坐标系下的位姿
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype=np.float64)

        self.time_begin_ = None
        self.time_end_ = None
        self.time_now_ = None

        self.node_state_ = self.NodeState.WAITING

        #self.target_groundtruth_ = 'rgbee'
        self.target_result_ = None
        self.is_result_received_ = False

        self.cmdstartPub_ = rospy.Publisher('/tello/cmd_start', Bool, queue_size=100)

        self.commandSub_ = rospy.Subscriber('/tello/cmd_string', String, self.commandCallback)
        self.poseSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.poseCallback)
        self.strSub_ = rospy.Subscriber('/tello/target_result', String, self.strCallback)
        self.simtimeSub_ = rospy.Subscriber('/clock', Clock, self.simtimeCallback)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.echoMessage()
            rate.sleep()

    def readYaml(self):
        yaml_path = self.ros_path_ + '/config/target.yaml'
        file_handle = open(yaml_path, 'r')
        cfg = file_handle.read()
        cfg_dict = yaml.load(cfg)
        self.target_groundtruth_ = cfg_dict['target_groundtruth_']

        pass

    def echoMessage(self):
        if self.node_state_ == self.NodeState.WAITING:
            os.system('clear')
            print('裁判机状态: 等待指令')
            print('')
            print('')
            pressAnyKeyExit('按任意键发送比赛开始信号，并开始计时。')
            cmd_start = Bool()
            cmd_start.data = 1
            self.cmdstartPub_.publish(cmd_start)
            self.node_state_ = self.NodeState.FLYING
            pass
        elif self.node_state_ == self.NodeState.FLYING:
            os.system('clear')
            print('裁判机状态: 正在比赛中')
            print('')
            self.printCompareResult()
            print('')
            if self.time_begin_ is not None:
                print('比赛开始时间: ' + rostime2str(self.time_begin_))
            if self.time_now_ is not None:
                print('当前时间:     ' + rostime2str(self.time_now_))
            if self.time_begin_ is not None:
                print('总用时:       ' + rostime2str(self.time_now_ - self.time_begin_))
            pass
        elif self.node_state_ == self.NodeState.LANDING:
            os.system('clear')
            print('裁判机状态: 接收到降落信号')
            print('')
            self.printCompareResult()
            print('')
            if self.time_begin_ is not None:
                print('比赛开始时间: ' + rostime2str(self.time_begin_))
            if self.time_now_ is not None:
                print('当前时间:     ' + rostime2str(self.time_now_))
            if self.time_begin_ is not None:
                print('总用时:       ' + rostime2str(self.time_now_ - self.time_begin_))
            pass
        elif self.node_state_ == self.NodeState.LANDED:
            os.system('clear')
            print('裁判机状态: 比赛结束')
            print('')
            self.printCompareResult()
            print('')
            if self.time_begin_ is not None:
                print('比赛开始时间: ' + rostime2str(self.time_begin_))
            if self.time_end_ is not None:
                print('比赛结束时间: ' + rostime2str(self.time_end_))
            if self.time_now_ is not None:
                print('当前时间:     ' + rostime2str(self.time_now_))
            if self.time_end_ is not None:
                print('总用时:       ' + rostime2str(self.time_end_ - self.time_begin_))
            pass
        else:
            pass
        pass

    def printCompareResult(self):
        print('目标识别结果正确答案: ' + self.target_groundtruth_)
        if self.target_result_ is None:
            print('尚未收到目标识别结果……')
        else:
            print('你的目标识别结果:     ' + self.target_result_)
            correct_results = 0
            if len(self.target_result_) == 5:
                for i in range(5):
                    if self.target_groundtruth_[i] == self.target_result_[i]:
                        correct_results += 1
            print('正确识别目标数:       ' + str(correct_results))

    def commandCallback(self, msg):
        cmd = msg.data
        cmdBuffer = cmd.strip().split()
        if cmdBuffer[0] == 'land' and self.node_state_ == self.NodeState.FLYING:
            self.node_state_ = self.NodeState.LANDING
        pass

    # 接收无人机位姿ground truth
    def poseCallback(self, msg):
        if 'iris' in msg.name:
            i = msg.name.index('iris')
            pose = msg.pose[i]
            self.t_wu_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.R_wu_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            if pose.position.z < 0.1 and self.node_state_ == self.NodeState.LANDING:
                self.node_state_ = self.NodeState.LANDED

    def strCallback(self, msg):
        # 只接受一次结果
        if not self.is_result_received_:
            self.target_result_ = msg.data
            self.is_result_received_ = True
        pass

    def simtimeCallback(self, msg):
        self.time_now_ = msg.clock
        if self.node_state_ == self.NodeState.FLYING and self.time_begin_ is None:
            self.time_begin_ = msg.clock
        if self.node_state_ == self.NodeState.LANDED and self.time_end_ is None:
            self.time_end_ = msg.clock

if __name__ == '__main__':
    jn = JudgeNode()

