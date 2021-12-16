#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import numpy as np
from enum import Enum
from threading import Lock
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

class telloControl:

    class FlightState(Enum):
        WAITING = 1
        FLYING = 2
        READY_TO_LAND = 3
        LANDING = 4

    def __init__(self):
        rospy.init_node('sim_tello_interface_node', anonymous=True)
        #无人机状态
        self.current_state_ = State()
        self.flight_state_ = self.FlightState.WAITING
        self.mavstateSub_ = rospy.Subscriber('/mavros/state', State, self.mavstateCallback)
        self.gazeboPoseSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazeboPoseCallback)  #gazebo world中各种模型的状态，包括name,pose={position(x,y,z)+orition(x,y,z,w)}和twist={linear(x,y,z)+angular(x,y,z)}

        #无人机在世界坐标系下的位姿
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype = 'float')

        #无人机的初始位姿，在第一次收到相关topic后初始化，用于进行Gazebo世界坐标与MAVROS本地坐标的转换
        self.t_init_ = np.zeros([3], dtype = 'float')
        self.yaw_init_ = 0.0  #角度制
        self.R_init_ = R.from_quat([0, 0, 0, 1])
        self.is_traj_local_init_ = False

        #封装实现tello控制函数
        self.cmdTello_  = rospy.Subscriber('/tello/cmd_string', String, self.cmdUpdate)
        self.cmdBuffer = None

        #接收指令最小间隔/s
        self.cmd_interval = 0.5
        self.last_cmd_ = rospy.Time.now()

        #无人机控制噪声
        self.ctrl_noise_std = 10
        self.truncation = 20

        #定时发送控制命令(至少2Hz)
        self.pub_interval_ = rospy.Duration(0.02)
        self.publishloop_timer_ = rospy.Timer(self.pub_interval_, self.publishloopCallback)

        #设置无人机状态
        self.arming_client_ = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.land_client_ = rospy.ServiceProxy('/mavros/cmd/land', CommandBool)
        self.set_mode_client_ = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        #用于发送起降命令和追踪飞行状态
        self.last_request_ = rospy.Time.now()
        self.offb_set_mode_ = SetMode()
        self.arm_cmd_ = CommandBool()
        
        #目标位置信息
        self.mutex_target_pose = Lock()
        self.uav_target_pose_local_ = PoseStamped()
        self.uav_target_pose_local_.header.seq = 1
        self.uav_target_pose_local_.header.frame_id = 'map'
        self.uav_target_pose_local_.header.stamp = rospy.Time.now();
        self.uav_target_pose_local_.pose.position.x = 0
        self.uav_target_pose_local_.pose.position.y = 0
        self.uav_target_pose_local_.pose.position.z = 1.5
        self.uav_target_pose_local_.pose.orientation.x = 0
        self.uav_target_pose_local_.pose.orientation.y = 0
        self.uav_target_pose_local_.pose.orientation.z = 0
        self.uav_target_pose_local_.pose.orientation.w = 1
        #当前姿态信息
        self.current_pose_ = PoseStamped()  #世界坐标
        self.current_pose_.header.seq = 1
        self.current_pose_.header.frame_id = 'map'
        self.current_pose_.pose.orientation.w = 1

        #发布目标位置
        self.uav_target_pose_local_pub_ = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)

        #在切换到offboard模式之前，必须先发送一些期望点信息到飞控中。 不然飞控会拒绝切换到offboard模式。
        rate = rospy.Rate(20);
        for i in range(10):
            self.uav_target_pose_local_pub_.publish(self.uav_target_pose_local_);
            rate.sleep()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    #更新无人机状态
    def mavstateCallback(self, msg):
        self.current_state_ = msg

    def gazeboPoseCallback(self, msg):
        if 'iris' in msg.name:
            i = msg.name.index('iris')
            self.current_pose_.header.stamp = rospy.Time.now()
            pose = msg.pose[i]
            self.current_pose_.pose = pose
            self.t_wu_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.R_wu_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            if not self.is_traj_local_init_:
                #第一次收到的世界坐标时记录无人机初始坐标
                self.t_init_ = self.t_wu_
                #(self.yaw_init_, pitch, roll) = self.R_wu_.as_euler('zyx', degrees = True)
                #self.R_init_ = self.R_wu_
            self.is_traj_local_init_ = True


    #更新控制命令
    def cmdUpdate(self, msg):
        cmd = msg.data
        self.cmdBuffer = cmd.strip().split()
        self.telloSDK()


    def telloSDK(self):
        try:
            if self.cmdBuffer is None:
                return

            if not self.current_state_.connected:
                return

            if (rospy.Time.now() - self.last_cmd_) < rospy.Duration(self.cmd_interval):
                #print("[CMD Failed]Send command later!")
                return

            if self.cmdBuffer[0] == 'takeoff':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdTakeoff()
            elif self.cmdBuffer[0] == 'land':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdLand()
            elif self.cmdBuffer[0] == 'up':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdUp()
            elif self.cmdBuffer[0] == 'down':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdDown()
            elif self.cmdBuffer[0] == 'left':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdLeft()
            elif self.cmdBuffer[0] == 'right':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdRight()
            elif self.cmdBuffer[0] == 'forward':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdForward()
            elif self.cmdBuffer[0] == 'back':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdBack()
            elif self.cmdBuffer[0] == 'cw':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdCw()
            elif self.cmdBuffer[0] == 'ccw':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdCcw()
            elif self.cmdBuffer[0] == 'stop':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdStop()
            elif self.cmdBuffer[0] == 'state':
                #print("CMD: "+self.cmdBuffer[0])
                self.last_cmd_ = rospy.Time.now()
                self.cmdState()
            else:
                pass

        except Exception as e:
            rospy.loginfo(e)

    #飞高1m
    def cmdTakeoff(self):
        if self.flight_state_ == self.FlightState.WAITING:
            if self.current_state_.mode != 'OFFBOARD': #and (rospy.Time.now() - self.last_request_ > rospy.Duration(2.0)):
                response = self.set_mode_client_(0, 'OFFBOARD')  #请求解锁
                if response.mode_sent:
                    rospy.loginfo('Offboard enabled')
                self.last_request_ = rospy.Time.now()
            
            if (not self.current_state_.armed): #and rospy.Time.now() - self.last_request_ > rospy.Duration(2.0):
                response = self.arming_client_(True)  #请求起飞
                if response.success:
                    rospy.loginfo('Vehicle armed')
                    self.flight_state_ = self.FlightState.FLYING
                self.last_request_ = rospy.Time.now()

            pose = self.current_pose_.pose
            #目标位姿
            target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
            x = pose.position.x
            y = pose.position.y
            z = 1.5
            target_position_global = np.array([x, y, z])
            #self.mutex_target_pose.acquire()
            self.updateTargetPose(target_position_global, target_orientation_global)
            #self.mutex_target_pose.release()
        
        else:
            pass


    def cmdLand(self):
        if self.flight_state_ == self.FlightState.FLYING:
            rospy.loginfo('Ready to land')
            if self.current_state_.mode != 'AUTO.LAND':
                response = self.set_mode_client_(0, 'AUTO.LAND')  #请求降落
                if response.mode_sent:
                    rospy.loginfo('Vehicle landing')
                    self.flight_state_ = self.FlightState.WAITING
                    self.already_takeoff = False
                self.last_request_ = rospy.Time.now()
        else:
            pass


    def cmdUp(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=20 and int(self.cmdBuffer[1])<=500) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                ctrl_noise = self.ctrl_noise_std * np.random.randn()
                ctrl_noise = np.maximum(ctrl_noise, -self.truncation)
                ctrl_noise = np.minimum(ctrl_noise, self.truncation)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z + int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                target_position_global = np.array([x, y, z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                #print("before cmd: ", current_position_global)
                #print("after cmd: ", target_position_global)
                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdDown(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=20 and int(self.cmdBuffer[1])<=500) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                ctrl_noise = self.ctrl_noise_std * np.random.randn()
                ctrl_noise = np.maximum(ctrl_noise, -self.truncation)
                ctrl_noise = np.minimum(ctrl_noise, self.truncation)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                x = pose.position.x
                y = pose.position.y
                z = max(0, pose.position.z - int(int(self.cmdBuffer[1])+ctrl_noise)/100.0)
                target_position_global = np.array([x, y, z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                #print("before cmd: ", current_position_global)
                #print("after cmd: ", target_position_global)
                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdLeft(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=20 and int(self.cmdBuffer[1])<=500) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                ctrl_noise = self.ctrl_noise_std * np.random.randn()
                ctrl_noise = np.maximum(ctrl_noise, -self.truncation)
                ctrl_noise = np.minimum(ctrl_noise, self.truncation)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                x = pose.position.x + np.cos(yaw/180*np.pi+np.pi/2)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                y = pose.position.y + np.sin(yaw/180*np.pi+np.pi/2)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                z = pose.position.z
                target_position_global = np.array([x, y, z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                #print("before cmd: ", current_position_global)
                #print("after cmd: ", target_position_global)
                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdRight(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=20 and int(self.cmdBuffer[1])<=500) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                ctrl_noise = self.ctrl_noise_std * np.random.randn()
                ctrl_noise = np.maximum(ctrl_noise, -self.truncation)
                ctrl_noise = np.minimum(ctrl_noise, self.truncation)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                x = pose.position.x + np.cos(yaw/180*np.pi-np.pi/2)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                y = pose.position.y + np.sin(yaw/180*np.pi-np.pi/2)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                z = pose.position.z
                target_position_global = np.array([x, y, z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                #print("before cmd: ", current_position_global)
                #print("after cmd: ", target_position_global)
                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdForward(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=20 and int(self.cmdBuffer[1])<=500) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                ctrl_noise = self.ctrl_noise_std * np.random.randn()
                ctrl_noise = np.maximum(ctrl_noise, -self.truncation)
                ctrl_noise = np.minimum(ctrl_noise, self.truncation)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                x = pose.position.x + np.cos(yaw/180*np.pi)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                y = pose.position.y + np.sin(yaw/180*np.pi)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                z = pose.position.z
                target_position_global = np.array([x, y, z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                #print("before cmd: ", current_position_global)
                #print("after cmd: ", target_position_global)
                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdBack(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=20 and int(self.cmdBuffer[1])<=500) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                ctrl_noise = self.ctrl_noise_std * np.random.randn()
                ctrl_noise = np.maximum(ctrl_noise, -self.truncation)
                ctrl_noise = np.minimum(ctrl_noise, self.truncation)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                x = pose.position.x - np.cos(yaw/180*np.pi)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                y = pose.position.y - np.sin(yaw/180*np.pi)*int(int(self.cmdBuffer[1])+ctrl_noise)/100.0
                z = pose.position.z
                target_position_global = np.array([x, y, z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                #print("before cmd: ", current_position_global)
                #print("after cmd: ", target_position_global)
                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdCw(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=1 and int(self.cmdBuffer[1])<=360) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                #print("yaw before: ",(yaw,pitch,roll))
                yaw -= int(self.cmdBuffer[1])
                #print("yaw after: ",(yaw,pitch,roll))
                target_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                target_orientation_global = R.from_euler('zyx',(yaw,pitch,roll), degrees = True)
                
                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdCcw(self):
        try:
            if len(self.cmdBuffer)==2 and (int(self.cmdBuffer[1])>=1 and int(self.cmdBuffer[1])<=360) and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                #print("before cmd: ", pose.position)
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
                #print("yaw before: ",(yaw,pitch,roll))
                yaw += int(self.cmdBuffer[1])
                #print("yaw after: ",(yaw,pitch,roll))
                target_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
                target_orientation_global = R.from_euler('zyx',(yaw,pitch,roll), degrees = True)

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()

                #print('-'*20)
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdStop(self):
        try:
            if len(self.cmdBuffer)==1 and self.flight_state_ == self.FlightState.FLYING:
                pose = self.current_pose_.pose
                #目标位姿
                target_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                target_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])

                #self.mutex_target_pose.acquire()
                self.updateTargetPose(target_position_global, target_orientation_global)
                #self.mutex_target_pose.release()
            else:
                self.cmdBuffer = None
        except Exception as e:
            rospy.loginfo(e)

    def cmdState(self):
        pose = self.current_pose_.pose
        current_position_global = np.array([pose.position.x, pose.position.y, pose.position.z])
        current_orientation_global = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        current_orientation_global = current_orientation_global.as_euler('zyx', degrees = True)
        #print("current position   : ", current_position_global)
        #print("current orientation: ", current_orientation_global)
        #print('-'*20)

    def updateTargetPose(self, target_position_global, target_orientation_global):
        (yaw, pitch, roll) = target_orientation_global.as_euler('zyx', degrees = True)
        yaw = yaw-self.yaw_init_
        while yaw<0:
            yaw += 360
        while yaw>=360:
            yaw -= 360
        target_position_local = np.matmul(self.R_init_.inv().as_dcm(), target_position_global-self.t_init_)
        target_orientation_local = R.from_euler('zyx',(yaw, pitch, roll), degrees = True).as_quat()

        self.uav_target_pose_local_.header.stamp = rospy.Time.now()
        self.uav_target_pose_local_.pose.position.x = target_position_local[0]
        self.uav_target_pose_local_.pose.position.y = target_position_local[1]
        self.uav_target_pose_local_.pose.position.z = target_position_local[2]
        self.uav_target_pose_local_.pose.orientation.x = target_orientation_local[0]
        self.uav_target_pose_local_.pose.orientation.y = target_orientation_local[1]
        self.uav_target_pose_local_.pose.orientation.z = target_orientation_local[2]
        self.uav_target_pose_local_.pose.orientation.w = target_orientation_local[3]

    def publishloopCallback(self, event):
        #if self.already_takeoff:
        self.uav_target_pose_local_pub_.publish(self.uav_target_pose_local_)


if __name__ == '__main__':
    #封装控制接口，与tello一致
    telloControl()
