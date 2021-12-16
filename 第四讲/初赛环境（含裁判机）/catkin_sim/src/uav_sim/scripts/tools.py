#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import os
import sys
import termios

def pressAnyKeyExit(msg): # copied from CSDN
    # 获取标准输入的描述符
    fd = sys.stdin.fileno()
    # 获取标准输入(终端)的设置
    old_ttyinfo = termios.tcgetattr(fd)
    # 配置终端
    new_ttyinfo = old_ttyinfo[:]
    # 使用非规范模式(索引3是c_lflag 也就是本地模式)
    new_ttyinfo[3] &= ~termios.ICANON
    # 关闭回显(输入不会被显示)
    new_ttyinfo[3] &= ~termios.ECHO
    # 输出信息
    sys.stdout.write(msg)
    sys.stdout.flush()
    # 使设置生效
    termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
    # 从终端读取
    os.read(fd, 7)
    # 还原终端设置
    termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)

def getPackagePath(package):
    p = os.popen('rospack find ' + package)
    path = p.read()
    path = path[0:-1]
    return path

def rostime2str(rostime):
    min = str(rostime.secs // 60).zfill(2)
    sec = str(rostime.secs % 60).zfill(2)
    nsec = str(rostime.nsecs // 10000000).zfill(2)
    str_result = min + ':' + sec + '.' + nsec
    return str_result
