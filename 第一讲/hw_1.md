# 作业一：ROS 基础学习
**前言**：本次作业旨在通过不同的任务使同学掌握ROS基础，包括工作环境、功能包的创建。
**预先要求**：ubuntu18.04＋ROS melodic

------

## 任务一：创建ROS工作空间（workspace，简称ws）
**目标**：本任务旨在讲解如何创建一个ROS工程。
打开终端并执行下列命令
``` 
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```
其中

1. catkin为ROS系统代码的编译器。

2. ```mkdir```。
```mkdir```为创建文件夹使用方式 mkdir <文件夹名>，默认只能创建一个文件夹。
```mkdir -p```添加选项```-p```可以直接创建多个层级文件夹。比如上述命令或直接创建工作空间catkin_ws以及该工作空间路径下的src两个文件夹。

3. ```catkin_make```。
```catkin_make```为编译器catkin的编译命令，catkin编辑器具体信息请参考[<u>http://wiki.ros.org/catkin</u>](http://wiki.ros.org/catkin)。

该指令运行的路径下必须含有src文件夹。```catkin_make```指令执行时会在src文件夹中自动查找CMakeLists.txt，并根据该文件内部命令编译文件，若src文件夹内没有CMakeLists.txt，```caktin_make会自动进行工作空间的初始化。

编译完成后catkin_make会在执行路径下自动生成devel，build两个文件夹。

**build**：编译文件夹，内部存储编译时生成的全部文件。

**devel**：development的缩写，代表软件的版本为内部开发版（与之相对的是安装版本install）开发时的全部文件与运行时的环境变量设置文件setup.<类型>

完成项目开发后，需要将编译好的文件的所在目录添加到系统的执行路径当中以使系统可以运行开发好的ros功能。

--------

## 任务二：ROS节点基础概念

本任务旨在实现使用ROS来开发一个应用。

首先安装一个轻量级的仿真器。安装命令如下：

打开一个终端
```
$ sudo apt-get install ros-melodic-ros-tutorials
$ roscore
```
保持该终端打开，并开启一个新终端（可以开一个新tab或者新的终端窗口）
```
$ rosnode list
```
以上代码中roscore会启动一个ROS master，而rosnode list会显示当前运行的ROS系统中运行的全部节点名称。

ROS master负责注册各个节点并配对已经注册的节点，一个形象的比喻为ROS master就像一个婚介所，需要传递消息的节点是来求偶的人。首先节点需要在婚介所ROS master中登记，然后婚介所才能根据需求将各个节点进行配对。配对后的节点之间可以直接传递消息，不再需要婚介所，即在ROS master完成节点的匹配后可以直接关闭运行roscore的终端而不会影响节点之间的通信。不同在于：一个节点可以配对多个其他节点。

Rosnode是ROS系统中与节点有关的命令。ROS节点是能够执行某种功能的模块。

Rosnode list命令的在终端运行后会显示结果/rosout，这表明ROS system中只运行了一个名称为/rosout的节点。/rosout节点实现的功能为收集日志，在收集和记录节点的调试输出时运行。

我们可以运行下面的指令来查看该节点的运行信息。
```
$ rosnode info /rosout
```
终端会显示下列信息
```
------------------------------------------------------------------------
Node [/rosout]
Publications:
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions:
 * /rosout [unknown type]

Services:
 * /rosout/get_loggers
 * /rosout/set_logger_level

contacting node http://machine_name:54614/ ...
Pid: 5092
```
其中Publications展示了节点发布出信息，Subscription展示了节点订阅的信息。Services展示了节点中提供的服务。
- 本例中/rosout发布的**话题(topic)**名称为/rosout_agg，这个话题传输的**消息(message，简写msg)**类型为rosgraph_msgs/Log，/rosout会将这个话题以广播的形式发送到ros master中的全部节点，但只有与/rosout配对的节点才会收到对应的消息。
- 本例子中/rosout订阅了/rosout的消息，消息类型为未知。
- 本例中/rosout提供了2个**服务(services，简写srv)**：/rosout/get_loggers 和/rosout/set_logger_level。提供的这两个服务可以被其他节点调用。

话题和服务是ROS系统中常用的通信方式（其他方式请自行学习）。
- 话题：话题只用于节点之间传递数据。传递的数据叫做**消息**。方式为发送节点(publisher)在ros master中注册话题发布者信息并以一个设定的频率广播话题，话题中发送的数据类型为消息(message)；接收节点(subscriber)在ros master中注册话题订阅者信息并也以一个设定的频率监听话题，每当监听到指定的话题时，调用函数处理话题传递的消息。这种信息传递方式称为**发布/订阅(publisher/subscriber)**。
- 服务：服务相当于节点之间传递应用程序接口(Application Programming Interface, API)。方式为服务器节点(server)在节点内设计一个服务函数，并将该服务函数注册到ros master中；客户节点(client)将服务的请求提交到ros master中，服务函数的调用传递的数据类型为服务(service)。这种信息传递方式称为**服务器/客户(server/client)**。

至此，我们区分两个概念：
1. 话题topic 和 服务 service是**通信类型**。
2. 消息(.msg文件) 和 服务(.srv文件) 是**数据类型**，是第一条中两个通信中传递数据的类型。两者的区分为：消息没有返回值而服务有返回值。

下面是一个示例消息文件person.msg
```
string name
uint8  sex
uint8  age 
 
uint8  unknown = 0 
uint8  male    = 1
uint8  female  = 2
```
下面是一个示例服务文件Person.srv
```
string name
uint8  age
uint8  sex

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2

---
string result
```

![话题通信-传递消息.png](https://upload-images.jianshu.io/upload_images/15871661-7c4c1b2a71626c58.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![服务通信-传递服务.png](https://upload-images.jianshu.io/upload_images/15871661-2d3ca92c529c1d9c.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

由该例可见，.srv就是比.msg多了返回值类型。在.srv的定义文件中，返回值和请求值通过三个横线分割。

最后，所有信息的传递都基于TCP/IP协议，这就意味着运行在电脑A上的节点node A可以调用另一台电脑上的node B中提供的服务API，也可以向其他多台电脑的多个节点发送话题。采用这种方案可以轻松实现远程控制机器人。

-----

## 任务三：ROS节点的运行与ROS功能包
实际生活中操作一个指定机器人需要很多功能，比如轮子驱动，摄像头信息处理等等，为了方便我们通常会将这种操作同一个实体的功能打包。ROS中也提供这个功能，即将多个功能相关的节点打包成一个**功能包package**。

功能包和功能包内部的节点通过下述命令调用。
```
$ rosrun [package_name] [node_name]
```
比如运行 turtlesim包中的 turtlesim_node 。
```
# 终端1
$ roscore
# 终端2
$ rosrun turtlesim turtlesim_node
# 终端3
$ rosnode list
```
终端2会启动额外的窗口

![turtle窗口.png](https://upload-images.jianshu.io/upload_images/15871661-316263c4255c5854.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

此时终端3中会返回
```
/rosout
/turtlesim
```
表示当前ROS系统中运行了两个不同的节点，名称为/rosout & /turtlesim。这两个节点的名称是在脚本内节点初始化时指定的。

此外，节点的名称也可以在启动时通过终端手动修改，比如
```
# 终端1
$ roscore
# 终端2
$ rosrun turtlesim turtlesim_node __name:=my_turtle
# 终端3
$ rosnode list
```
此时终端3中的输出为
```
/my_turtle
/rosout
```
其中节点名称被修改为了/my_turtle

我们也可以使用下面的指令来测试某个节点是否启动：
```
$ rosnode ping my_turtle
```
如果节点/my_turtle启动后，会返回下面类似的消息
```
rosnode: node is [/my_turtle]
pinging /my_turtle with a timeout of 3.0s
xmlrpc reply from http://<host_name>:<port_num>/     time=1.152992ms
xmlrpc reply from http://<host_name>:<port_num>/     time=1.120090ms
xmlrpc reply from http://<host_name>:<port_num>/     time=1.700878ms
xmlrpc reply from http://<host_name>:<port_num>/     time=1.127958ms
```
----

# 任务四：理解ROS话题Topic

运行下列程序
```
# terminal 1
$ roscore
# terminal 2
$ rosrun turtlesim turtlesim_node
# terminal 3
$ rosrun turtlesim turtle_teleop_key
```
终端3会显示下列信息：
```
Reading from keyboard
---------------------------
Use arrow keys to move the turtle. 'q' to quit.
```
在终端3中使用键盘的方向键操作乌龟移动。

新启动的turtle_teleop_key便是提供"键盘"操作乌龟功能的节点，该节点与turtlesim_node就是通过话题topic进行通信。具体来讲，turtle_teleop_key通过话题发布(publish)按键，而turtlesim订阅对应的话题并接收按键，两者之间的通信流程可以通过rqt_graph进行可视化。其安装方式如下：
```
$ sudo apt-get install ros-melodic-rqt
$ sudo apt-get install ros-melodic-rqt-common-plugins
```

启动方式为：
```
# 打开新终端
rqt_graph
```
![乌龟键盘控制和节点移动之间的消息订阅图.png](https://upload-images.jianshu.io/upload_images/15871661-9059e90d9f36e61a.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

或者

![乌龟键盘控制和节点移动之间的消息订阅图.png](https://upload-images.jianshu.io/upload_images/15871661-df80e1e771f71dfe.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

请注意左上角可以通过Hide选项屏蔽一些我们不关系的topic和节点
将鼠标移动到/teleop_turtle上，则rqt_graph便会被高亮成
- 红色：话题的发送者（publisher）
- 绿色：话题名称（topic）

![乌龟键盘控制和节点移动之间的消息订阅图-高亮相关信息.png](https://upload-images.jianshu.io/upload_images/15871661-e889051c0ed98be1.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

或者

![乌龟键盘控制和节点移动之间的消息订阅图-高亮相关信息.png](https://upload-images.jianshu.io/upload_images/15871661-092982a5d6e6089f.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


ROS系统中操作话题（topic）的相关指令包为rostopic。通过在终端中输入
```
rostopic -h
```
我们可以获得rostopic的各种用法。如下：
```
rostopic bw     display bandwidth used by topic
rostopic echo   print messages to screen
rostopic hz     display publishing rate of topic    
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type
```
此外也可以通过输入rostopic<空格>后双击Tab键查看候选子命令。
```
$ rostopic 
bw    echo  find  hz    info  list  pub   type 
```

查看rostopic传输的数据是ROS系统中debug最常用的一种方式：
```
rostopic echo <topic_name>
```
此指令用于回放(echo)<topic_name>中的数据。
另外，当<topic_name>输出的信息较多时，也可以通过下方指令将输出的信息手动输出到一个日志文件中。
```
rostopic echo <topic_name>  > /tmp/log
```
这个指令将rostopic echo <topic_name>的输出信息存入文件/tmp/log

在这个示例程序中，为了探究turtle是怎么动起来的，最直观的做法就是查看turtle_teleop_key究竟传给乌龟什么信息。
故我们需要查看两者传递的话题内容。（话题名称就是rqt_graph红色高亮的内容）
通过
```
$ rostopic echo /turtle1/cmd_vel
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
linear: 
  x: 0.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 2.0
---
```
启动rostopic echo之后，重新启动rqt_graph或者点击刷新按钮，其中多了rostopic_14587_xxxx，该节点便是rostopic echo启动的节点。
![启动rostopic echo后的rqt_graph](https://upload-images.jianshu.io/upload_images/15871661-9ac4eda5011143d7.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

或者

![启动rostopic echo后的rqt_graph.png](https://upload-images.jianshu.io/upload_images/15871661-7969cd702f3bfd55.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


此外可以通过rostopic list -h可以查看rostopic list的所有子命令的功能如下：
```
Usage: rostopic list [/topic]

Options:
  -h, --help            show this help message and exit
  -b BAGFILE, --bag=BAGFILE
                        list topics in .bag file
  -v, --verbose         list full details about each topic
  -p                    list only publishers
  -s                    list only subscribers
```

```
$ rostopic list -v

Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /rosout [rosgraph_msgs/Log] 4 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/cmd_vel [geometry_msgs/Twist] 2 subscribers
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /statistics [rosgraph_msgs/TopicStatistics] 1 subscriber
```
该命令可以显示出当前ROS系统中运行的全部topic及其相关发布者和订阅者的数量。

最后，debug还需要知道topic传递的数据类型以及名称。topic传递数据类型的名称可以通过
```
rostopic type <topic_name>
```
查看，比如
```
$ rostopic type /turtle1/cmd_vel
geometry_msgs/Twist
```
而该消息定义的数据类型通过下面命令查看
```
$ rosmsg show geometry_msgs/Twist
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```
由于geometry_msgs数据类型是ros内建的消息类型，可在ros官网上找到每一个变量的含义。如[官网](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)所说
，geometry_msgs/Twist定义了速度的平移和旋转分解。

Tips：ROS开源社区提供的搜索并不好用，推荐的方式为bing.com的国际版中搜索 ros <msg名称>，通常第一个候选就是官网链接。

其实，大多数debug时我们只需要查看每个topic内部传输的消息msg的数据类型。按照上面的方式，查看消息类型需要执行两个指令，略显麻烦，下面可以用一个指令直接显示话题topic内部传输消息msg的数据类型。
```
$ rostopic type /turtle1/cmd_vel | rosmsg show
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

至此，我们便可以解释乌龟移动的原因。
turtle_teleop_key节点通过/turtle1/cmd_vel话题(topic)发送geometry_msgs/Twist类型的消息(msg)，消息中记录了乌龟车当前速度的分解，从而接收节点按照速度要求移动乌龟车。

----

# 任务五：手动发布ROS话题Topic
启动乌龟仿真节点。
```
# terminal 1
$ roscore
# terminal 2
$ rosrun turtlesim turtlesim_node
# terminal 3
$ rosrun turtlesim turtle_teleop_key
```
通过rostopic pub命令，我们可以在rostopic中手动发布一次topic。
```
rostopic pub <topic_name> <msg_type> <args>
```
其中pub后的-1代表只发送此话题一次，<topic_name>为话题名称，<msg_type>为传递的消息类型，<args>为消息内部变量的赋值。其中<args>的输入格式可以通过双击tab键自动补全，然后修改对应的数值来实现。
比如
```
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
注意：该命令无法直接复制运行。当命令行中输入话题名称后（geometry_msgs/Twist），输入空格+双击Tab键，后面的内容会自动补全成空值，我们按键盘左右键移动到对应位置修改变量数值即可。


自动补全的内容是按照yaml命令行格式组织，详情请查阅官方文档[http://wiki.ros.org/ROS/YAMLCommandLine](http://wiki.ros.org/ROS/YAMLCommandLine)
至此，我们尝试一下手动发送topic使turtle移动
```
# terminal 1
$ roscore
# terminal 2
$ rosrun turtlesim turtlesim_node

$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 2.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
publishing and latching message for 3.0 seconds
```

注意：该命令无法直接复制运行，可以复制到Twist处，随后终端中双击Tab补全后续内容，并修改响应数据。

你会看到乌龟车向其左手边移动。

此外，我们也可以以设定的频率发布乌龟车移动的话题（使用pub -r <frequency>)子选项
```
$ rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 2.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0"
```
此指令以1Hz的频率发布乌龟的移动话题。此数据可以使乌龟逆时针转圈。原理如下：

linear中y等于2，制定了乌龟当前线速度有y轴正方向分量（而机器人坐标系下，前方为x正向，左手边为y轴正向，上方为z轴正向）

angular中z=2，代表乌龟车的角速度存在绕z轴正向旋转的分量，绕轴旋转正方向按照右手螺旋定则确定（右手拇指指向z轴正方向，四个手指指向的旋转方向就是绕z轴旋转的正向）。

注意：turtle运动只在二维平面中，因此没有沿z轴方向的线速度&绕x轴的角速度&绕y轴的角速度。

此时再刷新rqt_graph我们可以看到下面的图片：
![rqt_graph:手动定时发送topic.png](https://upload-images.jianshu.io/upload_images/15871661-5a57030b4de47dff.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

或者

![rqt_graph:手动定时发送topic.png](https://upload-images.jianshu.io/upload_images/15871661-a25765db80a4bd7d.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


 另外，我们还可以回放乌龟的位姿信息。
```
$ rostopic echo /turtle1/pose
x: 5.34955310822
y: 4.52340221405
theta: -1.26353096962
linear_velocity: 2.0
angular_velocity: 2.0
---
x: 5.3797287941
y: 4.53405189514
theta: -1.23153102398
linear_velocity: 2.0
angular_velocity: 2.0
---
x: 5.40954875946
y: 4.5456609726
theta: -1.19953095913
linear_velocity: 2.0
angular_velocity: 2.0
---
```
乌龟在各个位置的坐标、角度，以及线速度&角速度数值都在此显示。

另外，我们可以查询某个话题topic发布的频率
```
$ rostopic hz /turtle1/pose
subscribed to [/turtle1/pose]
average rate: 62.471
	min: 0.015s max: 0.017s std dev: 0.00050s window: 60
average rate: 62.479
	min: 0.015s max: 0.017s std dev: 0.00051s window: 122
average rate: 62.484
	min: 0.015s max: 0.017s std dev: 0.00050s window: 185
```
由此可得发布的频率大致为62Hz。

上述显示内容均为文字，并不直观。ROS系统中还提供图形可视化工具rqt_plot，其使用方式如下：
```
rosrun rqt_plot rqt_plot
```
若rqt_plot显示的窗口很小而且拖拽时报错```ValueError: bottom cannot be >= top```，则是由于rqt_plot调用的matplotlib版本过低，运行下面代码升级python中的matplotlib。

```
$ sudo apt-get install python-pip
$ pip install --upgrade matplotlib
```

在上方的输入文本框中输入需要监听的话题topic名称。
![rqt_plot显示画图精度.png](https://upload-images.jianshu.io/upload_images/15871661-967b2c82b5eade2e.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

----

# 任务六：理解ROS的服务Service

启动乌龟仿真节点。
```
# terminal 1
$ roscore
# terminal 2
$ rosrun turtlesim turtlesim_node
# terminal 3
$ rosrun turtlesim turtle_teleop_key
```
在任务二中我们提到：服务也是节点node之间的一种通信方式，由节点A提供服务函数API，节点B调用API并向节点A发送请求，待节点A接收到请求并处理后发送返回值给节点B。

由于服务也属于通信方式，所以与话题的操作颇为类似。

ROS系统中服务service的操作都包含在rosservice指令中，子命令如下：
```
$ rosservice -h
Commands:
	rosservice args	print service arguments
	rosservice call	        call the service with the provided args
	rosservice find	        find services by service type
	rosservice info	        print information about service
	rosservice list	        list active services
	rosservice type	print service type
	rosservice uri	        print service ROSRPC uri

Type rosservice <command> -h for more detailed usage, e.g. 'rosservice call -h'
```
我们也可以查看当前ROS系统中运行的服务列表
```
$ rosservice list
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/rqt_gui_py_node_31592/get_loggers
/rqt_gui_py_node_31592/set_logger_level
/rqt_gui_py_node_794/get_loggers
/rqt_gui_py_node_794/set_logger_level
/rqt_gui_py_node_844/get_loggers
/rqt_gui_py_node_844/set_logger_level
/spawn
/teleop_turtle/get_loggers
/teleop_turtle/set_logger_level
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

同样，我也可以查看服务service中传递的服务类型(.srv)
```
rosservice type <service_name>
```
比如
```
$ rosservice type /clear
std_srvs/Empty
```
std_srvs/Empty内部定义的数据类型可以通过下述命令列出
```
$ rossrv show std_srvs/Empty 
---

```

如终端所示，std_srvs/Empty的数据类型为空，三个横线上方为请求request（由客户client传给服务器server的数据类型）, 三个横线下方是响应response（由服务器server进行函数处理后返回客户client）。在该类型中，请求数据和返回数据都为空代表此服务不需要传入参数而且没有返回数值。

同样，上述两部操作可以合并为一个指令。
```
rosservice type <service_name> | rossrv show
```

定义好了服务之后，可以通过下面格式进行服务的调用：
```
rosservice call <service_name> <args>
```
其中<service_name>是服务的名称，即rosservice list中显示的各种服务名称。

<args>为传入参数，当没有传入参数时可以留空。

接下来我们调用其中ROS系统中turtlesim提供的/clear服务，该服务的功能为去除turtle移动留下的白色轨迹，我们首先先在终端3中操作turtle胡乱移动几段距离
接下来开启新终端，运行
```
$ rosservice call /clear
```
发现turtle所有移动过的白色轨迹全部被清空。

接下来我们再玩一下另外一个服务/spawn。
先来看一下该服务service传递的服务类型(.srv)。
```
$ rosservice type /spawn | rossrv show
float32 x
float32 y
float32 theta
string name
---
string name
```
此服务的功能为产生一个新的乌龟，的请求request传入的数据为坐标x, y, theta以及乌龟名称name。返回值为生成的乌龟名称。

那么我们就生仔搞一个新的乌龟。
```
$ rosservice call /spawn 2 2 0.2 ""
name: "turtle2"
```
![诞生了一只新乌龟.png](https://upload-images.jianshu.io/upload_images/15871661-76f105bb6310293e.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
命令行返回了诞生的新乌龟的名称“turtle2”

----

# 任务七：理解ROS的参数
ROS中的参数由ROS Parameter Server进行统一管理。所有运行在ROS系统中的节点均可以在运行时访问参数服务器中的参数。
不过需要注意：虽然参数服务器中的参数可以在ROS运行时动态修改，但其**并非**为**高性能**动态修改而设计，而是**配置参数**。

ROS参数服务器(ROS Parameter Server)是运行在ROS Master中用于管理参数的模块。参数服务器可以存储的数据类型有integers, floats, boolean, dictionaries和lists。其使用YAML markup语言的语法，具体而言： 1 是 integer, 1.0 是 float, "one"是string, true是boolean, [1, 2, 3]是integer的list,{a: b, c: d}是一个dictionary。

参数服务器的操作都集成于rosparam指令，具体子指令为
```
$ rosparam -h
rosparam is a command-line tool for getting, setting, and deleting parameters from the ROS Parameter Server.

Commands:
	rosparam set	    set parameter
	rosparam get	    get parameter
	rosparam load	    load parameters from file
	rosparam dump   dump parameters to file
	rosparam delete  delete parameter
	rosparam list	    list parameter names
```

启动乌龟仿真节点。
```
# terminal 1
$ roscore
# terminal 2
$ rosrun turtlesim turtlesim_node
# terminal 3
$ rosrun turtlesim turtle_teleop_key
```
同样可以通过下述命令查看当前ROS系统中运行的参数列表。

```
$ rosparam list
/rosdistro
/roslaunch/uris/host_192_168_31_42__33689
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```
可以设置某个参数的数值/获得某个参数的数值
```
rosparam set <parameter_name>
rosparam get <parameter_name>
```
修改参数，比如
```
$ rosparam set /turtlesim/background_r 150
$ rosservice call /clear
```
我们可以发现，颜色由默认的蓝色变为紫色。

![改变背景颜色参数.png](https://upload-images.jianshu.io/upload_images/15871661-c3cd22b07cc110af.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

读取参数，比如
```
$ rosparam get /turtlesim/background_g 
86
```

也可以通过rosparam get / 查看整个参数服务器中存储的参数
```
$ rosparam get / 
rosdistro: 'melodic

  '
roslaunch:
  uris: {host_192_168_31_42__33689: 'http://192.168.31.42:33689/'}
rosversion: '1.14.9

  '
run_id: dc0c767c-f4ca-11ea-886f-d8cb8af58252
turtlesim: {background_b: 255, background_g: 86, background_r: 150}
```

参数服务器还支持导出当前参数服务器中的参数到外部文件中 与  从外部的文件中读取参数如下：
```
rosparam dump <file_name> <namespace>
rosparam load <file_name> <namespace>
```
其中<namespace>的含义为将所有的参数的名称前面增加前缀，如果留空则保留参数服务器中参数或者文件中参数的原始名称。

比如
```
$ rosparam dump params.yaml
```
此命令会将当前Parameter server中的参数导出到运行路径下

读取参数比如
```
$ rosparam load params.yaml copy_turtle
$ rosparam get /copy_turtle/turtlesim/background_b
255
```
可以发现，读取的参数前增加了copy_turtle的命名空间。

----

# 任务八：使用rqt_console & 日志的危险等级
rqt_console就像是一个rqt的IDE一样，里面集成了rqt的各种功能模块的显示。
运行方式如下：
```
rosrun rqt_console rqt_console
```
启动的rqt_console如下图所示：
![rqt_console.png](https://upload-images.jianshu.io/upload_images/15871661-47aac4c3cb393537.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

比如我们运行
```
rosrun turtlesim turtlesim_node
```
![运行turtlesim_node后rqt_console中显示的消息](https://upload-images.jianshu.io/upload_images/15871661-d103581f771f7a3c.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

并且启动新终端输入下方指令使乌龟朝着一个固定的方向移动。
```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
最终乌龟将撞墙，系统报warn信息如下：
![乌龟撞墙后系统报警告信息.png](https://upload-images.jianshu.io/upload_images/15871661-ab4830850d568991.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

警告信息的等级分为以下几个级别：
- Fatal
- Error
- Warn
- Info
- Debug

其中Fatal是最重要的等级，而debug是最低的等级。通过选择第二栏中的不同等级按钮，第一个显示框内可以只显示对应等级的消息。

----

## 任务九：ros_launch基础
在之前的任务中，我们通过如下方式启动乌龟仿真节点。

```
# terminal 1
$ roscore
# terminal 2
$ rosrun turtlesim turtlesim_node
# terminal 3
$ rosrun turtlesim turtle_teleop_key
```

该方式启动很繁琐，而且需要打开三个独立的终端或者tab。roslaunch就是将上述节点启动方式简化为单个命令。

```
$ cd ~/catkin_ws
$ catkin_make
$ cd ./src
$ catkin_create_pkg turtle_launch rospy roscpp std_msgs
$ cd turtle_launch
$ mkdir launch && cd launch
$ gedit start_turtle.launch
```

将下列代码放入start_turtle.launch

```
<launch>
<node pkg="turtlesim" type="turtlesim_node" name="turtle1" />
<node pkg="turtlesim" type="turtle_teleop_key" name="cmd_vel"/>
</launch>
```

```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

通过上述代码我们便创建了一个名为turtle_launch的功能包，此功能包的中包含一个start_turtle.launch启动文件，此文件中启动turtlesim_node 和 键盘控制程序 两个节点。
因此通过运行

```
# 终端1
$ roslaunch turtle_launch start_turtle.launch
```

我们便可以在终端1中控制乌龟的运动。
注意：在launch文件执行前并没有启动roscore，这是因为roslaunch会自动为我们启动roscore。

在上述代码中
1. roslaunch启动文件的命令格式为：

```
$ roslaunch <package> <filename.launch>
```

<package>为功能包名称
<filename.launch>为启动文件名称

2. source ~/catkin_ws/devel/setup.bash命令将创建的功能包路径写入搜索路径使其可发现。
3. launch文件内部采用xml格式，请自行搜索xml文件的格式说明已获得更深入的理解。
在这个launch文件中
1. <launch> 和 </launch>两个标签内部的文件即launch文件的有效的代码。
2. <node blablabla />：由<node 起始到 />结束包含的内容为节点的有效代码。
3. 节点中pkg代表功能包的名字， type代表运行c++程序对应的可执行文件的名称， name为节点的名称。

我们再尝试一个roslaunch的程序如下：
```
$ gedit ~/catkin_ws/src/turtle_launch/launch/mimic_turtle.launch
```
并在文件中撰写如下代码

```
<launch>
<group ns="turtlesim1">
<node pkg="turtlesim" type="turtlesim_node" name="sim" />
</group>

<group ns="turtlesim2">
<node pkg="turtlesim" type="turtlesim_node" name="sim" />
</group>

<node pkg="turtlesim" name="mimic" type ="mimic">
<remap from="input" to="turtlesim1/turtle1" />
<remap from="output" to="turtlesim2/turtle1" />
</node>

</launch>
```

运行这个写好的roslaunch程序

```
$ roslaunch turtle_launch mimic_turtle.launch
```

ROS会弹出两个乌龟仿真器窗口，我们控制turtle1转圈，然后观察turtle2的行为是否与turtle1相同。

```
$ rostopic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0"

```

注意：该命令无法直接复制运行，可以复制到Twist处，随后终端中双击Tab补全后续内容，并修改响应数据。

![turtle2 mimic turtle1.png](https://upload-images.jianshu.io/upload_images/15871661-1cc644ba8efb0dab.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

相应的rqt_graph如下，即turtlesim2是由mimic节点控制。

![turtle2 mimic turtle1 rqt_graph.png](https://upload-images.jianshu.io/upload_images/15871661-d069a28c0af89d49.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

或者

![turtle2 mimic turtle1 rqt_graph.png](https://upload-images.jianshu.io/upload_images/15871661-9f7ddfdd10ff26a4.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

------

## 任务十：创建ROS信息msg & 创建ROS服务数据类型srv

在之前的任务中，我们使用的都是ROS内建的数据类型，有时ROS内建的数据类型并不能满足我们的需要，这时就需要我们自主设计数据类型。

正如之前的任务所讲，数据类型包括消息.msg和服务.srv两种

.msg: 就是描述ROS消息message的数据类型，写好的.msg文件会生成c++和python中所需要的数据结构。

.srv: 就是描述ROS中服务的数据类型，与.msg的不同在于，.srv包括请求request和响应response两部分，两部分通过---区分。
在实际使用中，我们自主设计的.msg需要放置在功能包目录下的msg文件夹中，.srv需要放置在功能包目录下的srv文件夹中。

.msg/.srv语言中可以使用下列字段之一：

- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]

除此之外，ROS系统中还有一种特殊的类型Header。Header中包括时间戳，该msg在ROS中的坐标信息。这个Head经常会出现在.msg文件中。

一个典型的.msg文件如下：
```
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```
每个内容都属于<数据类型> <名称>

一个典型的.srv文件如下：
```
int64 A
int64 B
---
int64 Sum
```
其中A和B是请求request，Sum是响应response。

接下来我们以一个具体的.msg创建&应用流程为例讲解自定义.msg的使用。
首先我们仍然在任务九创建的工作空间内进行操作

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg self_msg_usage rospy roscpp std_msgs
$ cd self_msg_usage
$ mkdir msg && cd msg
$ gedit student.msg
# 将下列代码放入student.msg
string first_name
string last_name
uint8 age
uint32 score
```

至此，我们设计好了自主的.msg文件。下面我们要生成该文件在c++以及python中的数据类型定义文件。
我们打开self_msg_usage功能包中的package.xml。

```
$ gedit ~/catkin_ws/src/self_msg_usage/package.xml 
```

添加下面两行

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
message_generation和message_runtime就是根据.msg生成c++与python数据类型文件所需要的ROS内建库。message_generation在编译时需要，因此为build_depend；而message_runtime是运行时需要，所以为exec_depend。

另外，还需要在CMakeLists.txt中增加message_generation&message_runtime的代码成为如下所示(其他原有内容不需要修改)。
```
$ gedit ~/catkin_ws/src/self_msg_usage/CMakeLists.txt
```
添加下面内容

```
find_package(catkin REQUIRED COMPONENTS
  ...
   message_generation
  ...
)

add_message_files(
  FILES
  student.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)

```
注意：有些时候可能发现，即便在功能包的package.xml和CMakeLists.txt没有添加对应的依赖也可能顺利运行，之所以能够顺利运行的原因是在同一个工作空间内的其他功能包添加了对应的依赖。但是出于保险，每一个功能包都要完整的填写所有的依赖包。

至此，我们已经添加好了全部的依赖包，编译后便可以直接查看生成的.msg的内容如下。
```
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
$ rosmsg show self_msg_usage/student.msg
string first_name
string last_name
uint8 age
uint32 score
```

至此，我们完成了msg的创建和使用，接下来我们同样以一个例子看学习srv的创建和使用。
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg self_srv_usage rospy roscpp std_msgs
$ cd self_srv_usage
$ mkdir srv && cd srv
$ gedit student_score.srv
```
#将下面内容写入student_score.srv
```
string first_name
string last_name
uint8 age
---
uint32 score
```
与.msg的创建十分类似，只是多了响应response的返回数据类型。

与msg的功能包设置相同，仍然需要添加对应的编译依赖。

```
$ gedit ~/catkin_ws/src/self_srv_usage/package.xml 
```
添加下面两行内容

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
同样，需要在CMakeLists.txt中修改添加对应内容如下所示：
```
$ gedit ~/catkin_ws/src/self_srv_usage/CMakeLists.txt 
```

```
find_package(catkin REQUIRED COMPONENTS
  ...
   message_generation
  ...
)

add_service_files(
  FILES
  student_score.srv
)

catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```
至此，我们已经添加好了全部的依赖包，编译后便可以直接查看生成的.srv的内容如下。
```
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
$ rossrv show self_srv_usage/student_score.srv
string first_name
string last_name
uint8 age
---
uint32 score
```

ROS系统中服务也可以通过rossrv show查看某个服务的具体内容。
```
rossrv show <service_type>
```
比如
```
$ rossrv show std_srvs/SetBool 
bool data
---
bool success
string message
```
此外，也可以在上述查看srv服务类型的命令中，还可以不指明功能包的名称，如果不指明srv所属的功能包，则会列出所有包含该名称srv的全部srv列表，如下所示：
```
$ rossrv show SetBool 
[std_srvs/SetBool]:
bool data
---
bool success
string message
```


------

## 任务十一：使用python撰写一个ROS话题topic通信
从之前的任务中，我们学习了两种通信方式话题topic和服务service的调用方式，以及两种通信方式下传输的数据类型.msg和.srv。但是我们目前还未涉及如何自定义ROS话题的发布者(publisher)/订阅者(subscriber)。以及服务service的服务器(server)/客户(client)。

接下来，我们首先基于python撰写一个话题的发布-订阅。
在ROS系统中，撰写的python程序需要统一放置在功能包路径下的scripts文件夹中。
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg python_topic rospy roscpp std_msgs
$ cd python_topic
$ mkdir scripts && cd scripts
$ gedit talker.py
```
将下面内容写入talker.py

```
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String 
def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
直接复制的内容可能没有缩进，若复制过来的内容没有缩进，需要手动添加缩进（每一个缩进为4个空格）。 

代码解读：
1.  ```#!/usr/bin/env python```
每一个采用python撰写的ros node在顶部都必须有这个声明。这行代码能够保证代码会以python形式运行。
2. 所有python编写的ros node必须import rospy，因为相关的ros库都在这个包中。
3. 由于python脚本后续内容使用了ROS内建的String类型，因此同样需要from std_msgs.msg import String
4. ```pub = rospy.Publisher('chatter', String, queue_size=10)```。该语句定义了一个发布名为/chatter话题的发布者(pub)，该话题传输的数据类型为String（ROS系统内建的类型std_msgs.msg.String），传输数据队列长度最大为10，即当系统中已经发布了10个未被订阅接受的消息时，再发布的消息会挤出最原始的消息。
5. ```rospy.init_node('talker', anonymous=True)```。该语句定义了这个python文件定义的节点名称为'talker?'后面的？是由```anonymous=True```指定而随机生成的数字，该数字的作用为防止该节点与其他节点重名。
注意：此处talker前不可以加slash "/"。
6. ```rate = rospy.Rate(10) # 10hz```。该语句创建了一个名为rate的Rate对象。通过循环中加入rate.sleep()函数，该循环可以实现每秒10次，即10Hz（只要每次循环处理时间不超过1/10s）。
7. 下方的循环是ros编程的模板，包括以下几步：

```
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
```

8. 检查```rospy.is_shutdown()```标志位然后开始进行工作。此标志位用于判断程序是否需要退出（比如用户是否在终端输入了ctrl-c）。若程序并未退出，调用pub将hello_str作为消息内容以名为/chatter的话题发布出去。执行完成发布任务后调用rate.sleep()来增长一次循环的时间以满足每秒执行10次的要求。其中```rospy.loginfo(hello_str)```具有三个功能：终端中显示hello_str字符串；将hello_str写入日志文件；将hello_str写到rosout。

注意：每个python脚本在创建后需要赋予其可执行权限

```
chmod  +x ./talker.py
```

至此，我们完成了发布者的创建。
订阅者的创建方式如下

```
$ cd ~/catkin_ws/src/python_topic/scripts
$ gedit listener.py
```

将下面内容写入listener.py，同样需要保持缩进。

```
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

注意：每个python脚本在创建后需要赋予其可执行权限

```
chmod  +x ./listener.py
```

代码解读：
1. 订阅者的绝大部分代码与发布者类型。核心代码 ```rospy.Subscriber("chatter", String, callback)```与发布者的区别在于，订阅者多了一个callback回调函数。这句代码声明该节点接受名为/chatter的话题（传输数据变量类型为std_msgs.msg.String)，并将接收到的String数据传入回调函数callback。
2. rospy.spin()用于防止节点执行过subscriber定义后退出，相反在该函数的作用下，只要节点不退出，subscriber会一直监听。
至此，我们设计好了发布者与订阅者，改变当前路径到工作空间根目录编译生成可执行文件，命令如下：

```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
# 终端1
$ roscore
# 终端2
$ rosrun python_topic talker.py 
# 终端3
$ rosrun python_topic listener.py
```


------

## 任务十二：使用python撰写一个ROS服务通信

ROS服务是另一种通信方式，整体结构与ROS消息十分相似。
首先定义需要传输的数据结构AddTwoInts.srv

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg python_service roscpp rospy std_msgs
$ cd  ~/catkin_ws/src/python_service
$ mkdir srv
$ gedit AddTwoInts.srv
#将下面信息输入
int64 a
int64 b
---
int64 sum
```

并按照先前任务增加对应的依赖关系，如下所示。

```
$ gedit ~/catkin_ws/src/python_service/package.xml 
```

添加下面两行内容

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

同样，需要在CMakeLists.txt中修改添加对应内容如下所示：

```
find_package(catkin REQUIRED COMPONENTS
  ...
   message_generation
  ...
)

add_service_files(
  FILES
  AddTwoInts.srv
)

catkin_package(
  ...
 CATKIN_DEPENDS message_runtime
  ...
)

generate_messages(
     DEPENDENCIES
     std_msgs
)
```

先来看客户client

```
$ mkdir -p ~/catkin_ws/src/python_service/scripts && cd  ~/catkin_ws/src/python_service/scripts
$ gedit add_two_ints_client.py
```

将下列写至python文件中。

```
#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from python_service.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```

完成撰写后赋予该脚本执行权限

```
sudo chmod +x scripts/add_two_ints_client.py
```

代码解读：与publisher主要的不同在于，服务的调用函数需要先定义在调用（定义调用名为add_two_ints的服务函数API，该服务传输的数据.srv类型为python_service.srv.AddTwoInts）。除此之外需要处理返回数值如下所示。

```
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
resp1 = add_two_ints(x, y)
return resp1.sum
```


再来看服务器server

```
$ cd  ~/catkin_ws/src/python_service/scripts
$ gedit add_two_ints_server.py
```

将下列写至python文件中。

```
#!/usr/bin/env python

from __future__ import print_function

from python_service.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

完成后赋予文件的可执行权限

```
chmod +x scripts/add_two_ints_server.py
```

代码解读：唯一与publisher的不同为 s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)。由于client会传入请求request，而且server需要计算返回值，因此需要回调函数。

设计完成后，编译检测功能可行性

```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
# 终端1
$ roscore
# 终端2
$ rosrun python_service add_two_ints_server.py
# 终端3
$ rosrun python_service add_two_ints_client.py 1 3 
Requesting 1+3
1 + 3 = 4
```

------

## 任务十三：ROS记录工具

ROS的记录工具rosbag允许我们存储当前ROS系统中的话题topic的信息。
具体操作如下：

终端1```roscore```

终端2```rosrun turtlesim turtlesim_node```

终端3```rosrun turtlesim turtle_teleop_key```

选择一个文件夹并创建对应的记录内容。

终端4

```
$ mkdir ~/bagfiles
$ cd ~/bagfiles
$ rosbag record -a
```

其中```record -a```选项代表存储当前运行的全部话题的名称。
启动上述终端后在终端3中移动turtle任意一段路径。控制turtle移动这段轨迹的话题topic将会被存储到bagfiles文件夹中的.bag文件中。

现在可以使用```ctrl-c```终止终端4停止记录。接下来分析rosbag记录的内容的具体信息。

```
rosbag info <your_bag_file>
```

通过上面的指令我们获得所记录bag文件的日期、版本、文件大小、时间长度、以及记录了哪些话题&话题传输的消息数目等信息。

下面体会一下bag文件如何使用：```rosbag play <your_bag_file>```
即（双击Tab键自动不全对应的bag文件名称。）

```
rosbag play ~/bagfiles/<name>.bag
```

比如
```
$ rosbag play -r 1 2020-09-14-09-22-23.bag
[ INFO] [1600046614.272979376]: Opening 2020-09-14-09-22-23.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
[RUNNING]  Bag Time: 1600046543.914829   Duration: 0.000000 / 12.790592
```

```waiting 0.2 seconds ...```的含义为：在真正发布bag文件中的消息之前，rosbag需要先发布一个发消息并等待0.2s。这个0.2s是用来通知系统中运行的节点此消息的存在，如果开始就直接发送bag中的消息的话可能会造成第一包的丢失。

```rosbag play -r <frequency>```选项用来设定发布消息的频率。

此外，我们也可以通过下面的指令只记录ROS系统中指定的消息。

```
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```

rosbag的用途之一即记录下当前ROS系统中传输的topic中数据内容以用于后续的分析。
下面就是rosbag中话题发布的消息的内容的一种查看方式。其原理为使用```rostopic echo [topic_name] > [file_name]```监听话题内容并将其输入到```<file_name>```中。

比如

```
rostopic echo /rosout > /tmp/log
```

同时使用```rosbag play <bag_file>```发布之前存储好的话题。

-----

## 任务十四：Python 控制乌龟车

至此，大家应该初步了解了ROS topic的机制。
现在，请大家做一个作业，练一练用 python 实现话题的发布。

编写节点 py_control.py 和 py_control.launch。

首先新建工作包 py_control

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg py_control rospy roscpp std_msgs geometry_msgs #因为要用到twist命令，所以需要导入 geometry_msgs
$ cd py_control
$ mkdir script launch
$ touch script/py_control.py launch/py_control.launch #新建 script/py_control.py launch/py_control.launch 文件
$ chmod +x script/py_control.py #添加可执行权限
```

编辑python文件： script/py_control.py:

```
#!/usr/bin/env python
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist

def controller():
    rospy.init_node('pycontroller', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #set topic name
    rate = rospy.Rate(0.3) # 0.3hz
    
    count = 0
    while not rospy.is_shutdown():
        if count % 4 == 0:
            control_cmd = Twist()
            control_cmd.linear.x  = 2 #使小车向前开
        if count % 4 == 1:
            control_cmd = Twist() #请修改Twist内容，使其左转
        if count % 4 == 2:
            control_cmd = Twist() #请修改Twist内容，使其继续向前
        if count % 4 == 3:
            control_cmd = Twist() #请修改Twist内容，使其右转
        count += 1
        rospy.loginfo(control_cmd)
        pub.publish(control_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
```

其中 py_control.launch 文件编写如下。请参考注释修改功能

```
<launch>
<!-- 先启动乌龟车项目 -->
<node pkg="turtlesim" type="turtlesim_node" name="turtle1" />

<!-- 再启动自己写的python节点 -->
<!-- <node pkg="turtlesim" type="py_control.py" name="pycontrol"/> -->
</launch>
```

等文件编写完成后， ```catkin_make``` 并且 ```source devel/setup.sh```任务空间。

然后

```
roslaunch py_control
py_control.launch
```

如果看到乌龟车按照你的计划正常运行，说明成功。请仔细理解你写的python文件中每一行代码（包括课组提供的代码框架）。如果你都理解了，就可以放学了。

-----

## 任务十五： 国庆自学

请参考网络学堂的实验指导书，自学ROS第二次课程《ros基础与TF树》。
并且完成相关作业，在10月16日晚提交实验报告。
