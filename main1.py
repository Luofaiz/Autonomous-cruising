#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import tf
import threading
import rospy
import sys
import math
from sensor_msgs.msg import LaserScan
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslib
from std_msgs.msg import String, Int32MultiArray  # 新增导入 Int32MultiArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from std_msgs.msg import String, Float32MultiArray
import ctypes
from ctypes.util import find_library
import os
from playsound import playsound
from ar_track_alvar_msgs.msg import AlvarMarkers
key = 0
F_list = []

global xy
global task_xy  
global qtn_list_xy
global qtn_list_task_xy
global pose_num_xy
global pose_num_task_xy
global yaw
global now_pose_X
global now_pose_Y




#xy          =[[0.16,-0.30,3.14], [3.0,-0.32,0], [3.03,-3.02,-0], [0.01, -2.95, 3.14],[0.5, -2.55, 3.14]]
xy          =[[0.339, -0.373,-3.112], [2.45,-0.75,0], [2.45,-2.55,0], [0.60,-2.50,3.14],[0.6,-2.50,3.14]]
#task_xy     =[[0.17,-1.22,3.14],[1.08,-0.25,3.14],[2.17,-0.27,0], [2.93,-1.14,0], [2.96,-2.15,0],[2.10,-3.04,0],[1.09,-3.0,3.14],[0.23,-2.10,3.14]]
task_xy     =[[0.65,-0.65,3.14],[0.65,-0.65,3.14],[2.45,-0.75,0], [2.45,-0.75,0], [2.45,-2.55,0],[2.45,-2.55,0],[0.60,-2.50,3.14],[0.6,-2.50,3.14]]
pid_stop_xy =[[0.93,0.566,270],[0.938,0.92,270],[0.946,-0.51,270], [0.956,0.494,270], [0.962,-0.958,270],[0.952,-0.596,270],[1.026,0.598,90],[1.022,0.948,90],[1.04,-0.512,90],
              [0.2,0.1,0],[-0.267,0.245,0],[0.209,1.601,0],[0.187,0.207,0]]

qtn_list_xy = []           #四元数列表
qtn_list_task_xy = []      #任务点四元数列表
pose_num_xy= len(xy)
pose_num_task_xy=len(task_xy)
yaw = 0
global move_base
now_pose_X=0
now_pose_Y=0


def play_voice_recognition_result(listen_back, listen_choose):
    """
    播报语音识别的结果
    参数：
    - listen_back: 终点数字
    - listen_choose: 运算法则代码
    """
    try:
        # 1. 播报"识别到终点为.mp3"
        playsound("识别到终点为.mp3")
        print("播报：识别到终点为")
        
        # 2. 播报终点数字对应的mp3
        playsound("数字"+str(listen_back) + ".mp3")
        print(f"播报：{listen_back}")
        
        # 3. 播报"点.mp3"
        playsound("点.mp3")
        print("播报：点")
        
        # 4. 播报"运算符为.mp3"
        playsound("运算符为.mp3")
        print("播报：运算符为")
        
        # 5. 根据运算法则代码播报对应的运算符
        if listen_choose == 112233:
            playsound("加法.mp3")
            print("播报：加法")
        elif listen_choose == 334455:
            playsound("减法.mp3")
            print("播报：减法")
        elif listen_choose == 556677:
            playsound("乘法.mp3")
            print("播报：乘法")
        elif listen_choose == 778899:
            playsound("除法.mp3")
            print("播报：除法")
        else:
            print(f"未知的运算法则代码：{listen_choose}")
            
    except Exception as e:
        print(f"播报语音识别结果时发生错误：{e}")

def number_to_chinese(number):
    """
    将数字转换为对应的汉字
    参数：
    - number: 数字
    返回值：
    - 对应的汉字字符串
    """
    chinese_numbers = {
        0: "零", 1: "一", 2: "二", 3: "三", 4: "四", 
        5: "五", 6: "六", 7: "七", 8: "八", 9: "九",
        10: "十", 11: "十一", 12: "十二", 13: "十三", 14: "十四",
        15: "十五", 16: "十六", 17: "十七", 18: "十八", 19: "十九",
        20: "二十"
        # 可以根据需要扩展更多数字
    }
    
    # 如果是整数，直接查找
    if isinstance(number, int) and number in chinese_numbers:
        return chinese_numbers[number]
    
    # 如果是浮点数，转换为整数（四舍五入）
    if isinstance(number, float):
        number = round(number)
        if number in chinese_numbers:
            return chinese_numbers[number]
    
    # 如果找不到对应的汉字，返回数字字符串
    return str(number)

def play_calculation_and_navigation(task1, task2, listen_choose, result):
    """
    播报计算过程和导航信息
    参数：
    - task1: 第一个检测到的信息
    - task2: 第二个检测到的信息  
    - listen_choose: 运算法则代码
    - result: 计算结果
    """
    try:
        print("开始播报计算过程和导航信息...")
        
        # 将数字转换为汉字
        task1_chinese = number_to_chinese(task1)
        task2_chinese = number_to_chinese(task2)
        result_chinese = number_to_chinese(result)
        
        # 1. 播报第一个信息的汉字.mp3
        playsound(f"{task1_chinese}.mp3")
        print(f"播报第一个信息：{task1_chinese}")
        
        # 2. 播报运算符.mp3
        if listen_choose == 112233:
            playsound("加.mp3")
            print("播报运算符：加法")
        elif listen_choose == 334455:
            playsound("减.mp3")
            print("播报运算符：减法")
        elif listen_choose == 556677:
            playsound("乘.mp3")
            print("播报运算符：乘法")
        elif listen_choose == 778899:
            playsound("除.mp3")
            print("播报运算符：除法")
        
        # 3. 播报第二个信息的汉字.mp3
        playsound(f"{task2_chinese}.mp3")
        print(f"播报第二个信息：{task2_chinese}")
        
        # 4. 播报"等于.mp3"
        playsound("等于.mp3")
        print("播报：等于")
        
        # 5. 播报计算结果对应的汉字.mp3
        playsound(f"{result_chinese}.mp3")
        print(f"播报计算结果：{result_chinese}")
        
        # 6. 播报"导航到.mp3"
        playsound("导航到.mp3")
        print("播报：导航到")
        
        # 7. 再次播报计算结果对应的汉字.mp3
        playsound(f"{result_chinese}.mp3")
        print(f"播报导航目标：{result_chinese}")
        
        # 8. 播报"点.mp3"
        playsound("点.mp3")
        print("播报：点")
        
        print("计算过程和导航信息播报完成")
        
    except Exception as e:
        print(f"播报计算过程和导航信息时发生错误：{e}")


# -----------pid函数及其参数------------
# 注：
# 1.
global w_kp 
global w_ki
global w_kd
global w_target
global w_e_all
global w_last_e

w_kp = 2 #2
w_ki = 0.001
w_kd = 0.005 #0
w_e_all = 0
w_last_e = 0
global x_f,x_b,y_l,y_r
x_f=0.0
x_b=0.0
y_l=0.0
y_r=0.0



def w_pid_cal(pid_target,dis):
    global w_kp 
    global w_ki
    global w_kd
    global w_e_all
    global w_last_e
    e = dis -pid_target
    #if e>-0.1 and e<0.1:
      #e = 0
    w_e_all = w_e_all+e
    pid = w_kp*e+w_ki*w_e_all+w_kd*(e-w_last_e)
    w_last_e = e
    return pid

global p_kp 
global p_ki
global p_kd
global p_e_all
global p_last_e
global p_pid
p_kp = -7
p_ki = 0
p_kd = -3
p_e_all = 0
p_last_e = 0
p_pid = 0

def p_pid_cal(pid_target,pose):
    global p_kp 
    global p_ki
    global p_kd
    global p_e_all
    global p_last_e
    ture_pose = (pose/3.14159265359*180.0+180.0)%360
    if pid_target==0:
        if ture_pose>0 and ture_pose<180:
            pid_target=0
        if ture_pose>180 and ture_pose<360:
            pid_target=360   
            
    e = ture_pose -pid_target
    # print(e) 
    p_e_all = p_e_all+e
    pid = p_kp*e+p_ki*p_e_all+p_kd*(e-p_last_e)
    #rospy.loginfo("e %f",e)	
    p_last_e = e
    return pid

global point_kp 
global point_ki
global point_kd
global point_e_all
global point_last_e
global point_pid
point_kp = -3
point_ki = 0
point_kd = 0
point_e_all = 0
point_last_e = 0
point_pid = 0

def point_pid(pid_target_x,ture):
    global point_kp 
    global point_ki
    global point_kd
    global point_e_all
    global point_last_e
    e = ture -pid_target_x
    point_e_all = point_e_all+e
    pid = point_kp*e+point_ki*point_e_all+point_kd*(e-point_last_e)	
    point_last_e = e
    return pid

def limt(limt,target):
    if limt>target:
        limt=target
    if limt<-target:
        limt=-target
    return limt

# ----------pid停车-------
def pid_stop2(target_x,target_y,target_yaw):
    global w_kp,w_ki,w_kd,w_e_all
    w_kp = 0.8 #1.5
    w_ki = 0.001 #0.005
    w_kd = 0.01 #0.01
    w_e_all=0
    count =0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    time = 0
    while not rospy.is_shutdown():
        rate_loop_pid.sleep()
        time+=1
        if target_x>0:
            pid_x = w_pid_cal(target_x,x_f)
            wich_x=x_f
        if target_x<0:
            pid_x = w_pid_cal(target_x,-x_b)
            wich_x=-x_b
        if target_y>0:
            pid_y = w_pid_cal(target_y,y_l)
            wich_y = y_l
        if target_y<0:
            pid_y = w_pid_cal(target_y,-y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw,yaw)
        #if abs(wich_x)<0.8:
            #speed.linear.y = 0
        #else:
        speed.linear.y = pid_y
        speed.linear.x = pid_x
        speed.angular.z = p_pid/180.0*3.14159265359
        w_e_all=limt(w_e_all,5)
        print("w_e_all:",w_e_all)
        print("wich_x:",wich_x,"wich_y:",wich_y)
        if time>=150:
            w_e_all=0
            break
        #if abs(wich_x-target_x)<=0.05 and abs(wich_y-target_y)<=0.07 and abs(target_yaw-(yaw/3.1415926*180+180))<=5:
            #w_e_all=5
            #w_ki = 0.001
        if abs(wich_x-target_x)<=0.03 and abs(wich_y-target_y)<=0.03 and abs(target_yaw-(yaw/3.1415926*180+180))<=3:
            w_e_all=0
        if count>=6:
            speed.linear.x = 0
            speed.linear.y = 0
            speed.linear.z = 0
            pid_vel_pub.publish(speed) 
            #rospy.sleep(0.5)
            w_e_all=0
            break
        pid_vel_pub.publish(speed) 

def pid_stop(target_x,target_y,target_yaw):
    global w_kp,w_ki,w_kd,w_e_all
    w_kp = 0.8 #1.5
    w_ki = 0 #0.005
    w_kd = 0.001 #0.01
    w_e_all=0
    count =0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    time = 0
    while not rospy.is_shutdown():
        rate_loop_pid.sleep()
        time+=1
        if target_x>0:
            pid_x = w_pid_cal(target_x,x_f)
            wich_x=x_f
        if target_x<0:
            pid_x = w_pid_cal(target_x,-x_b)
            wich_x=-x_b
        if target_y>0:
            pid_y = w_pid_cal(target_y,y_l)
            wich_y = y_l
        if target_y<0:
            pid_y = w_pid_cal(target_y,-y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw,yaw)
        #if abs(wich_x)<0.8:
            #speed.linear.y = 0
        #else:
        speed.linear.y = pid_y
        speed.linear.x = pid_x
        speed.angular.z = p_pid/180.0*3.14159265359
        w_e_all=limt(w_e_all,5)
        print("w_e_all:",w_e_all)
        print("wich_x:",wich_x,"wich_y:",wich_y)
        if time>=150:
            w_e_all=0
            break
        #if abs(wich_x-target_x)<=0.05 and abs(wich_y-target_y)<=0.07 and abs(target_yaw-(yaw/3.1415926*180+180))<=5:
            #w_e_all=5
            #w_ki = 0.001
        if abs(wich_x-target_x)<=0.1 and abs(wich_y-target_y)<=0.1 and abs(target_yaw-(yaw/3.1415926*180+180))<=5:
            w_e_all=0
        if count>=3:
            speed.linear.x = 0
            speed.linear.y = 0
            speed.linear.z = 0
            pid_vel_pub.publish(speed) 
            #rospy.sleep(0.5)
            w_e_all=0
            break
        pid_vel_pub.publish(speed) 

def pid_go(target_x,target_y,target_yaw):
    global point_kp, vision_result, dis_trun_off,point_ki
    global w_kp,w_ki,w_kd
    w_kp = 2 #2
    w_ki = 0
    w_kd = 0.008#0
    count =0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    while not rospy.is_shutdown():
        if target_x>0:
            pid_x = w_pid_cal(target_x,x_f)
            wich_x=x_f
        if target_x<0:
            pid_x = w_pid_cal(target_x,-x_b)
            wich_x=-x_b
        if target_y>0:
            pid_y = w_pid_cal(target_y,y_l)
            wich_y = y_l
        if target_y<0:
            pid_y = w_pid_cal(target_y,-y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw,yaw)
        if abs(target_x-wich_x)<0.2 and abs(target_y-wich_y)<0.2:
            speed.linear.x = 0
            speed.linear.y = 0
        else:
            speed.linear.y = pid_y
            speed.linear.x = pid_x
        speed.angular.z = p_pid/180.0*3.14159265359
        pid_vel_pub.publish(speed)   
        rate_loop_pid.sleep()
        #print("x_f:",x_f,"y_l:",y_l)
        #print(abs(target_yaw-yaw/3.1415926*180))
        if vision_result != 0:
            # rospy.sleep(0.3)
            # thread_dis.join()
            break


def pid_go2(target_x,target_y,target_yaw):
    global vision_result
    global w_kp,w_ki,w_kd,w_e_all
    print("--------开始pid_go2--------")
    w_kp = 2 #2
    w_ki = 0
    w_kd = 0.01 #0
    count =0
    w_e_all=0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    while not rospy.is_shutdown():
        if target_x>0:
            pid_x = w_pid_cal(target_x,x_f)
            wich_x=x_f
        if target_x<0:
            pid_x = w_pid_cal(target_x,-x_b)
            wich_x=-x_b
        if target_y>0:
            pid_y = w_pid_cal(target_y,y_l)
            wich_y = y_l
        if target_y<0:
            pid_y = w_pid_cal(target_y,-y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw,yaw)

        if abs(target_x-wich_x)<0.2 and abs(target_y-wich_y)<0.2:
            speed.linear.x = 0
            speed.linear.y = 0
        else:
            if abs(wich_x)>0.55:
                speed.linear.y = 0.03*pid_y
            else:
                speed.linear.y = pid_y
            speed.linear.x = pid_x
            speed.angular.z = p_pid/180.0*3.14159265359

        pid_vel_pub.publish(speed)       
        rate_loop_pid.sleep()
    
        if vision_result != 0:
            w_e_all=0

            # rospy.sleep(0.3)
            # thread_dis.join()
            break

    print("--------结束pid_go2--------")


def pid_turn(target_x,target_y,target_yaw):
    global point_kp 
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(10)
    speed = Twist()
    while not rospy.is_shutdown():
        p_pid = p_pid_cal(target_yaw,yaw)
        speed.linear.x = 0
        speed.linear.y = 0
        speed.angular.z = 0.4*p_pid/180.0*3.14159265359
        pid_vel_pub.publish(speed)   
        rate_loop_pid.sleep()
        print(abs(target_yaw-yaw/3.1415926*180))
        if abs(target_yaw-(yaw/3.1415926*180+180))<=10:
            speed.angular.z = 0
            pid_vel_pub.publish(speed)   
            #rospy.sleep(0.3)
            break

# -----------pid函数及其参数------------

# -----------获取视觉消息---------------
global ar_vision
ar_vision = 0


global ai_vision
ai_vision = 0
def ar_callback(msg):
    global ar_vision
    if len(msg.markers):
        print("----------------------------------ar")
        print(msg.markers[0].id)
        ar_vision = msg.markers[0].id

def vision_ar():
    rospy.Subscriber('/ar_pose_marker',AlvarMarkers,ar_callback,queue_size=10) #订阅视觉话题
    rospy.spin()
    
    

def ai_callback(msg):
    global ai_vision
    
    print("++++++++++++++++++++++++++++++++++++ai")
    print("ai回复",msg.data)
    ai_vision = int(msg.data)

def vision_ai():
    rospy.Subscriber('/vision_result',String,ai_callback,queue_size=20) #订阅视觉话题
    rospy.spin()


# -----------获取视觉消息---------------
    
global listen_choose
global listen_back 
listen_choose = 0
listen_back = 0
def voice_callback(msg):
    global listen_choose , listen_back
    if listen_choose ==0 and listen_back==0:
        listen_back = msg.data[0]
        listen_choose = msg.data[1]
    print(listen_back,listen_choose)


def voice_back():
    rospy.Subscriber('/target',Int32MultiArray,voice_callback,queue_size=10) #订阅视觉话题
    rospy.spin()
#------------------- 雷达话题订阅线程 ---------------- 

global scan_data
scan_data = []

def get_valid_distance(scan, start_index, direction, angle_resolution):
    """
    从指定的起始角度开始，以给定的方向和角度分辨率寻找有效的激光数据，
    并计算修正后的距离。

    参数：
    - scan: 激光雷达扫描数据
    - start_index: 起始角度索引
    - direction: 搜索方向（1 表示顺时针，-1 表示逆时针）
    - angle_resolution: 角度分辨率（每个索引代表的角度）

    返回值：
    - 修正后的有效距离
    """
    max_angle = len(scan.ranges)
    for i in range(max_angle):
        index = (start_index + i * direction) % max_angle
        if scan.ranges[index] != float('inf'):
            distance = scan.ranges[index]
            angle = np.radians((index - start_index) * angle_resolution)
            distance_corrected = distance * np.cos(angle)
            return distance_corrected
    return float('inf')

def get_laserscan(scan):    
    """
    激光雷达回调函数，计算前后左右方向的有效激光距离并进行修正。

    参数：
    - scan: 激光雷达扫描数据
    """
    global x_f, x_b, y_l, y_r, yaw, scan_data
    scan_data = scan.ranges
    
    front_index = 360  # 前方角度索引
    angle_resolution = 0.5  # 每个索引代表的角度（假设为0.5度）

    # 找到有效距离并进行修正
    x_f = get_valid_distance(scan, front_index, 1, angle_resolution)  # 从前方开始向右搜索
    x_b = get_valid_distance(scan, 0, 1, angle_resolution)  # 从后方开始向右搜索
    y_l = get_valid_distance(scan, 540, -1, angle_resolution)  # 从左侧开始向左搜索
    y_r = get_valid_distance(scan, 180, 1, angle_resolution)  # 从右侧开始向右搜索
    
    #print("x_f:", x_f, "x_b:", x_b, "y_l:", y_l, "y_r:", y_r)


def laser_listen():
    rospy.Subscriber('/scan', LaserScan,get_laserscan,queue_size=7)
    rospy.spin()

#------------------- 雷达话题订阅线程 ---------------- 
       

# -----------------------pid回家--------------------------
def pid_loop():
        print("---------启动回家--------")
        global yaw
        global w_kp,w_ki,w_kd,w_e_all
        w_e_all=0
        w_kp = 0.8 #2
        w_ki = 0
        w_kd = 0.00 #0
        pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
        rate_loop_pid=rospy.Rate(7)
        speed = Twist()
        while not rospy.is_shutdown():
            pid_x = w_pid_cal(0.155,x_f)  
            #pid_y = w_pid_cal(-0.35,-y_r)
            pid_y = w_pid_cal(0.155,y_l)
            p_pid = p_pid_cal(0,yaw)
            print("x_f",x_f)
            print("y_l",y_l)
            print("yaw",yaw)
            speed.linear.x = pid_x
            speed.linear.y = pid_y
            speed.angular.z = p_pid/180.0*3.14159265359
            pid_vel_pub.publish(speed)   
            rate_loop_pid.sleep()
            if abs(x_f-0.1)<0.18 and abs(y_l-0.1)<0.18 and abs(0-(yaw/3.1415926*180+180))<=5:
                speed.linear.x = 0
                speed.linear.y = 0
                pid_vel_pub.publish(speed)
                break   
        print("---------结束回家--------")

# -----------------------pid回家--------------------------
       
# ---------------vioce 语音播报-----------------



def play_voice(number):
    playsound(str(number)+".mp3")


def play_voice_begin(numbers):
    print("获取的任务目标为:"+str(target_point_arr))
    numbers=sorted(numbers)
    playsound(str(numbers[0])+"_"+str(numbers[1])+"_"+str(numbers[2])+".mp3")
    print("播放文件为："+str(numbers[0])+"_"+str(numbers[1])+"_"+str(numbers[2])+".mp3")
# ---------------vioce 语音播报-----------------

# -----------------导航点----------------------

# 1.本区域共三个函数导航实时距离获取，发布导航点，控制导航点发布
# 2.控制导航点发布部分默认是逐一发布点列表中的点，需要请注释并自行更改


# 发布目标点

# 控制导航点发布
global count_dis_times
count_dis_times=0
def dis_func(x, y,min_dis):
    global dis_trun_off,now_pose_X,now_pose_Y,distance, vision_result,target_point_arr,times_voice_play,count_dis_times
    print("dis_func函数已启动:"+str(count_dis_times)+"次")
    count_dis_times+=1
    # lock = threading.RLock()
    dis_fun_rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        dis_fun_rate.sleep()
        car_to_map_x=now_pose_X
        car_to_map_y=now_pose_Y
        # 计算小车到目标点的距离
        distance = pow(pow( car_to_map_x - x, 2) + pow(car_to_map_y - y, 2), 0.5)
        print("distance:"+str(distance))
        if distance<min_dis:
            print("reach_goal")
            #rospy.sleep(1)
            break
    


def goals(x, y, i):
    min_dis=0.07
    global qtn_list_xy,move_base,dis_trun_off,now_pose_X,pid_stop_vision_stop_flag,goal_vision_weight_flag
    target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_xy[i][0], qtn_list_xy[i][1], qtn_list_xy[i][2], qtn_list_xy[i][3]))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_point
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    rospy.loginfo("Going to: " + str(target_point))
    print("goal")
    move_base.send_goal(goal)

def goals_task(x, y, i):
    min_dis=0.07
    global qtn_list_task_xy,move_base,dis_trun_off,now_pose_X
    target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_task_xy[i][0], qtn_list_task_xy[i][1], qtn_list_task_xy[i][2], qtn_list_task_xy[i][3]))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_point
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    rospy.loginfo("Going to: " + str(target_point))
    print("goal")
    move_base.send_goal(goal)

def send_None_goal():
    global move_base
    goal = MoveBaseGoal()
    move_base.send_goal(goal)
    
# ---------------vioce 语音播报-----------------

global times_voice_play
times_voice_play=0

def play_voice(number):
    global times_voice_play
    playsound(str(number)+".mp3")

# ---------------vioce 语音播报-----------------





# ----------------- init ---------------------

# 1.处理点列表的四元数并放在新的列表
# 2.连接move_base

# --------------
def now_pose_xy():
    global now_pose_X,now_pose_Y,yaw
    now_pose=rospy.Rate(10)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        now_pose.sleep()
        try:
            (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            # 小车坐标
            now_pose_X=trans[0]
            now_pose_Y=trans[1]
            euler = tf.transformations.euler_from_quaternion(rot)
            yaw = euler[2]   # 第三个元素是yaw角
        except Exception as e:
            print("tf loading.........")

# --------------
def voice_wakeup_publisher():
    
    # 创建发布者，发布到 /voiceWakeup 话题，消息类型为 String
    pub = rospy.Publisher('/voiceWakeup', String, queue_size=10)
    
    playsound("比赛开始.mp3")
    # 获取用户输入
    user_input = input("请输入1开始启动: ")

    if user_input == "1":
        # 创建消息并发布
        msg = String()
        msg.data = "1"
        pub.publish(msg)
        rospy.loginfo("已发布消息: %s", msg.data)
        #playsound("比赛开始.mp3")
        rospy.sleep(1.5)

        # 等待 /target 话题的反馈
        rospy.loginfo("等待语音识别...")
        
    else:
        rospy.loginfo("error")

def init_fun():
    #转换点
    global qtn_list_xy,qtn_list_task_xy,pose_num_xy,pose_num_task_xy,move_base
    for i in range(pose_num_xy):
        qtn = tf.transformations.quaternion_from_euler(0,0,xy[i][2])
        qtn_list_xy.append(qtn)
    i=0
    for i in range(pose_num_task_xy):
        qtn = tf.transformations.quaternion_from_euler(0,0,task_xy[i][2])
        qtn_list_task_xy.append(qtn)


    #连接move_base—actition
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Request to connect to move_base server")
    rospy.loginfo("Be connected successfully")
    
    thread_lidar = threading.Thread(target=laser_listen)                         
    thread_lidar.start() 
     
    thread_ar = threading.Thread(target=vision_ar)                         
    thread_ar.start() 

    thread_ai = threading.Thread(target=vision_ai)                         
    thread_ai.start()

    thread_voice = threading.Thread(target=voice_back)                         
    thread_voice.start()
    
    thread_now_pose = threading.Thread(target=now_pose_xy)                         
    thread_now_pose.start() 
    
    
            

def vision_to_get_task(t_long):
    global ar_vision, ai_vision
    ai_vision = 66
    ar_vision = 0

    t = 0
    sure_required = 0
    rate_loop_pid=rospy.Rate(10)
    rospy.set_param('/top_view_shot_node/im_flag', 1)
    task_vision = 88
    #rospy.sleep(3)
    while not rospy.is_shutdown():
        
        if ai_vision != 66 and not ar_vision: 
            task_vision = ai_vision
            print("ai_vision",ai_vision)
        else :
            if ar_vision ==0:
                task_vision = 88
            else:
                task_vision = ar_vision
            print("ar_vision",ar_vision)
                
        t+=1
        if t>=t_long:
            print("超时")
            task_vision=999
            break
        if task_vision!=88:
            print("========",task_vision)
            break
        rate_loop_pid.sleep()

    return task_vision

def tts_speak(params_str):
    try:
        params = params_str.split(',')
        param1 = int(float(params[0].strip()))  # 先转浮点数再转整数
        param2 = int(float(params[1].strip()))  # 先转浮点数再转整数
        param3 = int(float(params[2].strip()))  # 先转浮点数再转整数
        param4 = int(float(params[3].strip()))  # 先转浮点数再转整数 - 修复这里
        
        if param3 == 112233:
            operation = "加"
        elif param3 == 334455:
            operation = "减"
        elif param3 == 556677:
            operation = "乘"
        elif param3 == 778899:
            operation = "除"
        else:
            operation = "未知"
        
        text = f"计算式{param1}{operation}{param2}等于{param4}"
        pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        rospy.sleep(0.5)  # 等待发布者注册到ROS Master
        
        # 创建发布数据
        msg = String()
        msg.data = text
        # 发布
        pub.publish(msg)
        rospy.loginfo("发布的数据：%s", msg.data)
        
    except ValueError as e:
        print(f"参数转换错误：{e}")
        print(f"接收到的参数字符串：{params_str}")
    except Exception as e:
        print(f"tts_speak函数发生错误：{e}")

def choose_count(num1, num2, choose):
    """
    修改计算函数，确保返回整数结果
    """
    try:
        if choose == 112233:
            result = num1 + num2
        elif choose == 334455:
            result = abs(num1 - num2)
        elif choose == 556677:
            result = num1 * num2
        elif choose == 778899:
            if num2 != 0:
                result = num1 / num2
                if result < 1 and num1 != 0:
                    result = num2 / num1
            else:
                result = num1  # 避免除零错误
        else:
            result = 0
            
        # 确保返回整数
        result = int(round(result))
        print(f"计算结果：{result}")
        return result
        
    except Exception as e:
        print(f"计算过程发生错误：{e}")
        return 0

# def choose_count(num1,num2,choose):
#     if choose == 112233 :
#         result = num1+num2
#     if choose == 334455:
#         result = abs(num1-num2)
#     if choose == 556677:
#         result = num1*num2
#     if choose == 778899:
#         result = num1/num2
#         if result < 1:
#             result = num2/num1
#     print("result",result)

#     return result  
        

# def tts_speak(params_str):
#     params = params_str.split(',')
#     param1 = int(params[0].strip())  # 第一个参数转为整数
#     param2 = int(params[1].strip())  # 第二个参数转为整数
#     param3 = int(params[2].strip())  # 第三个参数转为整数
#     param4 = int(params[3].strip())  # 第四个参数转为整数
    
#     if param3 == 112233:
#         operation = "加"
#     elif param3 == 334455:
#         operation = "减"
#     elif param3 == 556677:
#         operation = "乘"
#     elif param3 == 778899:
#         operation = "除"
#     else:
#         operation = None  # 或返回 "未知"
    
#     text = f"计算式{param1}{operation}{param2}等于{param4}"
#     pub = rospy.Publisher('/voiceWords', String, queue_size=10)
#     rospy.sleep(0.5)  # 等待发布者注册到ROS Master
    
#     # 创建发布数据
#     msg = String()
#     msg.data = text
#     # 发布
#     pub.publish(msg)
#     rospy.loginfo("发布的数据：%s",msg.data)


def play_recognition_result(number):
    """播报识别结果的专用函数"""
    if number != 88 and number != 999:  # 88是默认值，999是超时
        try:
            playsound(f"识别结果为{number}.mp3")
            print(f"播报识别结果：{number}")
        except Exception as e:
            print(f"播报识别结果失败：{e}")
            # 如果没有专门的识别结果音频，回退到数字音频
            try:
                playsound(f"{number}.mp3")
                print(f"回退播报数字：{number}")
            except Exception as e2:
                print(f"播报数字也失败：{e2}")

def task_1():
    global listen_choose
    
    # 移动到第一个检测位置
    pid_stop(-0.50,1.0,180)
    pid_turn(0,0,360)
    rospy.set_param('/top_view_shot_node/im_flag', 1)
    
    # 检测第一个位置的信息
    task1 = vision_to_get_task(150)
    print("==========================",task1)
    
    # 播报第一个检测结果
    if task1 != 88 and task1 != 999:
        print(f"第一个位置检测到：{task1}")
        play_recognition_result(task1)
    else:
        print("第一个位置未检测到有效信息")
    
    # 移动到第二个检测位置
    pid_turn(0,0,270)
    pid_stop(0.4,1.0,270)
    
    # 检测第二个位置的信息
    task2 = vision_to_get_task(150)
    print("==========================",task2)
    
    # 播报第二个检测结果
    if task2 != 88 and task2 != 999:
        print(f"第二个位置检测到：{task2}")
        play_recognition_result(task2)
    else:
        print("第二个位置未检测到有效信息")
    
    # 计算结果
    end1_task_stop = choose_count(task1,task2,listen_choose)
    print("----------------------------------",end1_task_stop)
    
    # 如果计算结果有效，执行后续动作
    if end1_task_stop < 999:
        # 播报计算过程和导航信息
        play_calculation_and_navigation(task1, task2, listen_choose, end1_task_stop)
        
        to_str = str(task1)+","+str(task2)+","+str(listen_choose)+","+str(end1_task_stop)
        #tts_speak(to_str)
        print(to_str)
        
        # 导航到计算结果对应的位置
        pid_stop2(pid_stop_xy[end1_task_stop-1][0],pid_stop_xy[end1_task_stop-1][1],pid_stop_xy[end1_task_stop-1][2])
        
        # 播报最终结果
        play_voice(end1_task_stop)
        print("reach_task1")
    else:
        print("计算结果无效，跳过后续动作")
        
def task_2():
    global listen_choose
    goals(xy[1][0],xy[1][1],1)
    dis_func(xy[1][0],xy[1][1],0.1)
    send_None_goal()
    pid_stop(0.40,1.0,180)
    task1 = vision_to_get_task(150)
    play_recognition_result(task1)
    pid_turn(0,0,270)
    pid_stop(0.4,-1.0,270)
    task2 = vision_to_get_task(150)
    play_recognition_result(task2)
    end1_task_stop=choose_count(task1,task2,listen_choose)
    if end1_task_stop<999:
        # 播报计算过程和导航信息
        play_calculation_and_navigation(task1, task2, listen_choose, end1_task_stop)
        
        to_str = str(task1)+","+str(task2)+","+str(listen_choose)+","+str(end1_task_stop)
        #tts_speak(to_str)
        print(to_str)
        #goals_task(task_xy[task-1][0],task_xy[task-1][1],task-1)
        #dis_func(task_xy[task-1][0],task_xy[task-1][1],0.08)
        #send_None_goal()
        
        pid_stop2(pid_stop_xy[end1_task_stop-1][0],pid_stop_xy[end1_task_stop-1][1],pid_stop_xy[end1_task_stop-1][2])
        #tts_speak(to_str)
        rospy.sleep(1)
        play_voice(end1_task_stop)
        #rospy.sleep(1)
        print("reach_task2")

def task_3():
    global listen_choose
    goals(xy[2][0],xy[2][1],2)
    dis_func(xy[2][0],xy[2][1],0.1)
    send_None_goal()
    pid_stop(0.40,-1.0,180)
    task1 = vision_to_get_task(150)
    play_recognition_result(task1)
    pid_turn(0,0,90)
    pid_stop(0.4,1.0,90)
    task2 = vision_to_get_task(150)
    play_recognition_result(task2)
    end1_task_stop=choose_count(task1,task2,listen_choose)
    if end1_task_stop<999:
        # 播报计算过程和导航信息
        play_calculation_and_navigation(task1, task2, listen_choose, end1_task_stop)
        
        to_str = str(task1)+","+str(task2)+","+str(listen_choose)+","+str(end1_task_stop)
        #tts_speak(to_str)
        print("play_voice")
        #goals_task(task_xy[task-1][0],task_xy[task-1][1],task-1)
        #dis_func(task_xy[task-1][0],task_xy[task-1][1],0.08)
        #send_None_goal()
        pid_stop2(pid_stop_xy[end1_task_stop-1][0],pid_stop_xy[end1_task_stop-1][1],pid_stop_xy[end1_task_stop-1][2])
        play_voice(end1_task_stop)
        #rospy.sleep(1)
        print("reach_task3")

def task_4():
    global listen_back
    goals(xy[3][0],xy[3][1],3)
    dis_func(xy[3][0],xy[3][1],0.1)
    send_None_goal()
    pid_stop2(pid_stop_xy[listen_back-1][0],pid_stop_xy[listen_back-1][1],pid_stop_xy[listen_back-1][2])
    play_voice(listen_back)
    #rospy.sleep(1)
    print("reach_task4")



# ----------------- init ---------------------
if __name__ == '__main__':
    # 初始化节点
  
    rospy.init_node('move_test', anonymous=True)
    init_fun()
    rospy.set_param('/top_view_shot_node/im_flag', 1)
    #a = input()
    #listen_back = 13
    #listen_choose = 112233
    voice_wakeup_publisher()
    while not listen_choose and not listen_back:
        print("语音识别....")
    print("终点：",listen_back,"预算法则：",listen_choose)

    print("开始播报语音识别结果...")
    play_voice_recognition_result(listen_back, listen_choose)
    print("语音识别结果播报完成，开始执行任务...")

    task_1()
    task_2()
    task_3()
    task_4()
    begin_time=rospy.Time.now()
print(" ________             __                ________ ")

finish_time=rospy.Time.now()
print("时间-------->%.2f",(finish_time-begin_time).to_sec())
