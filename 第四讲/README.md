## 第四讲
仿真环境

本章资料有：

2021年初赛赛题：公布本课程的初赛的任务要求和得分细则；

系统集成与初赛环境使用：讲解无人机&机器人的系统集成框架，简单介绍初赛仿真环境的开发；

仿真环境：认识仿真环境界面和飞行控制流程等；

初赛环境（含裁判机)

环境配置附件、虚拟机下载链接：https://cloud.tsinghua.edu.cn/d/7dcd7b370b98484e8b8a/?p=%2F&mode=list

### 仿真环境介绍
操作系统:Ubuntu 18.04
– 相关软件:
• ROS Melodic:机器人操作系统;
• Gazebo 9.0.0:三维机器人仿真平台，能够模拟室内/室外等各种复杂环境中的机器人;
• PX4 v1.9.2:自动驾驶仪固件，可用于驱动无人机;
• MAVLink:消息传输协议, 用于地面控制终端(地面站)与无人机之间进行通信。

配置好仿真环境后，可以运行如下命令，先认识仿真界面:
<code>roslaunch uav_sim arena_test_py.launch</code>
该命令会启动Rviz和Gazebo两个窗口。Rviz为ROS自带的图形化工具，下图中A区域为订阅的一些topic信息，B区域为image类型的消息.
<img width="1093" alt="image" src="https://user-images.githubusercontent.com/74605431/140716781-d44e43cf-0574-47dc-9154-8366b4fdffb4.png">





### 控制流程
– 无人机飞行流程如下图所示:
• Gazebo完成物理场景的模拟，通过发布话题的方式输出场景中各种物体的状态信息，以及无人机的位姿信息和拍摄的图像等;
• 无人机控制程序需要根据收集的数据进行信息融合并设计控制算法;
• 控制程序借助于MAVROS向无人机发送飞行指令(但是在仿真环境中，这一部分已经封装为tello的控制接口)，由PX4完成指令的执行。

<img width="715" alt="image" src="https://user-images.githubusercontent.com/74605431/140717295-e86bf5d4-d14f-4d40-be7d-04fdcd8b1a7f.png">




### 实验要求
– 利用前面的基本操作，集成完整的无人机控制流程: 
1. 结合tello的控制接口，控制无人机从指定位置起飞; 
2. 识别模拟火情标记(红色);
3. 穿过其下方对应的窗户，并在指定位置降落。



### 课程视频
https://cloud.tsinghua.edu.cn/d/85be4160128d4eef8f46/
