#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

noisy_state_topic = '/tello/states'

class addStateNoise:
    def __init__(self):
        rospy.init_node('sim_pose_publisher_node', anonymous=True)
        #在真实位置上加入噪声
        self.gazeboPoseSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.addPoseNoiseCallback)
        self.noisyPosePub_ = rospy.Publisher(noisy_state_topic, PoseStamped, queue_size=100)

        #获取噪声标准差参数
        self.noise_position = rospy.get_param('/pose_publisher/noise_position')                 #WGN标准差
        self.noise_orientation = rospy.get_param('/pose_publisher/noise_orientation')

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    def addPoseNoiseCallback(self, msg):
        if 'iris' in msg.name:
            i = msg.name.index('iris')
            pose = msg.pose[i]
            pose.position.x += self.noise_position*(2*np.random.rand()-1)
            pose.position.y += self.noise_position*(2*np.random.rand()-1)
            pose.position.z += self.noise_position*(2*np.random.rand()-1)
            pose.orientation.x += self.noise_orientation*(2*np.random.rand()-1)
            pose.orientation.y += self.noise_orientation*(2*np.random.rand()-1)
            pose.orientation.z += self.noise_orientation*(2*np.random.rand()-1)
            pose.orientation.w += self.noise_orientation*(2*np.random.rand()-1)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = pose

            self.noisyPosePub_.publish(pose_stamped)


if __name__ == '__main__':
    
    #真实状态加噪声
    addStateNoise()
    
