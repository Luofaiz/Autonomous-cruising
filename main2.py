#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# System and ROS imports reorganized
import rospy, sys, os, math, argparse, threading, ctypes
import tf, actionlib, roslib, tf2_ros, numpy as np
from ctypes.util import find_library
from playsound import playsound

# Message type imports
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, TransformStamped
from std_msgs.msg import Int32, String, Int32MultiArray, Float32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
#from ar_track_alvar_msgs.msg import AlvarMarkers

# ============================================================================
# Global Configuration and State Variables
# ============================================================================


nav_waypoint_coords = [[0.339, -0.373, -3.112], [2.45, -0.75, 0], [2.45, -2.55, 0], 
                       [0.60, -2.50, 3.14], [0.6, -2.50, 3.14]]

parking_target_coords = [[0.940, 0.556, 270], [0.928, 0.912, 270], [0.934, -0.516, 270], 
                        [0.942, 0.486, 270], [0.946, -0.957, 270], [0.948, -0.592, 270], 
                        [1.024, 0.596, 90], [1.022, 0.936, 90], [1.030, -0.507, 90],
                        [0.2, 0.1, 0], [-0.251, 0.233, 0], [0.199, 1.617, 0], [0.190, 0.199, 0]]

# System state variables
nav_quaternions_list = []
nav_point_count = len(nav_waypoint_coords)
robot_orientation_yaw, robot_pos_x, robot_pos_y = 0, 0, 0
move_base_action_client = None
counting_reaches = 0

# Sensor data globals
lidar_front_distance, lidar_back_distance = 0.0, 0.0
lidar_left_distance, lidar_right_distance = 0.0, 0.0
complete_laser_scan_data = []

# Voice and vision detection globals
#ar_marker_detection_result, ai_vision_detection_result = 0, 0
voice_operation_command, voice_target_destination = 0, 0

# PID controller parameters for distance control
dist_proportional_gain, dist_integral_gain, dist_derivative_gain = 2, 0.001, 0.005
dist_accumulated_error, dist_previous_error = 0, 0

# PID controller parameters for angular control  
angular_proportional_gain, angular_integral_gain, angular_derivative_gain = -7, 0, -3
angular_accumulated_error, angular_previous_error, angular_control_output = 0, 0, 0

# PID controller parameters for point control
point_proportional_gain, point_integral_gain, point_derivative_gain = -3, 0, 0
point_accumulated_error, point_previous_error, point_control_output = 0, 0, 0

# ============================================================================
# Audio and Speech Functions
# ============================================================================

def execute_precision_docking_v1(target_x_coord, target_y_coord, target_orientation_angle):
    """Standard precision parking version 1 - 低精度版本"""
    global dist_proportional_gain, dist_integral_gain, dist_derivative_gain, dist_accumulated_error
    
    dist_proportional_gain, dist_integral_gain, dist_derivative_gain = 0.8, 0, 0.001
    dist_accumulated_error = 0
    success_verification_count = 0
    
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    control_frequency = rospy.Rate(7)
    robot_command = Twist()
    actual_x_position, actual_y_position = 0, 0
    timeout_counter = 0
    
    while not rospy.is_shutdown():
        control_frequency.sleep()
        timeout_counter += 1
        
        if target_x_coord > 0:
            x_pid_result = compute_distance_pid_control(target_x_coord, lidar_front_distance)
            actual_x_position = lidar_front_distance
        if target_x_coord < 0:
            x_pid_result = compute_distance_pid_control(target_x_coord, -lidar_back_distance)
            actual_x_position = -lidar_back_distance
        if target_y_coord > 0:
            y_pid_result = compute_distance_pid_control(target_y_coord, lidar_left_distance)
            actual_y_position = lidar_left_distance
        if target_y_coord < 0:
            y_pid_result = compute_distance_pid_control(target_y_coord, -lidar_right_distance)
            actual_y_position = -lidar_right_distance
            
        angle_pid_result = compute_angular_pid_control(target_orientation_angle, robot_orientation_yaw)
        
        robot_command.linear.y = y_pid_result
        robot_command.linear.x = x_pid_result
        robot_command.angular.z = angle_pid_result / 180.0 * 3.14159265359
        
        dist_accumulated_error = constrain_value_within_bounds(dist_accumulated_error, 5)
        
        if timeout_counter >= 150:
            dist_accumulated_error = 0
            break
            
        # 低精度检查: ±10cm位置容差, ±5度角度容差
        x_tolerance_ok = abs(actual_x_position - target_x_coord) <= 0.1
        y_tolerance_ok = abs(actual_y_position - target_y_coord) <= 0.1
        angle_tolerance_ok = abs(target_orientation_angle - (robot_orientation_yaw / 3.1415926 * 180 + 180)) <= 5
        
        if x_tolerance_ok and y_tolerance_ok and angle_tolerance_ok:
            dist_accumulated_error = 0
            success_verification_count += 1
            
        if success_verification_count >= 3:  # 只需要连续成功3次
            robot_command.linear.x = 0
            robot_command.linear.y = 0
            robot_command.linear.z = 0
            velocity_publisher.publish(robot_command)
            dist_accumulated_error = 0
            break
            
        velocity_publisher.publish(robot_command)

def vocalize_single_number(numeric_value):
    """Play audio for a single number"""
    try:
        playsound(str(numeric_value) + ".mp3")
    except Exception as e:
        print(f"播放音频失败: {numeric_value}.mp3, 错误: {e}")

def broadcast_detection_outcome(detected_number):
    """Announce detection results"""
    if detected_number != 88 and detected_number != 999:
        try:
            playsound(f"识别结果为{detected_number}.mp3")
            print(f"检测到: {detected_number}")
        except Exception as primary_error:
            try:
                playsound(f"{detected_number}.mp3")
                print(f"检测到: {detected_number}")
            except Exception as secondary_error:
                print(f"播报数字失败：{secondary_error}")

# ============================================================================
# Mathematical and Utility Functions  
# ============================================================================

def constrain_value_within_bounds(input_value, maximum_bound):
    """Limit value within specified bounds"""
    if input_value > maximum_bound:
        input_value = maximum_bound
    if input_value < -maximum_bound:
        input_value = -maximum_bound
    return input_value

# ============================================================================
# PID Control Functions
# ============================================================================

def compute_distance_pid_control(target_dist, current_dist):
    """Calculate PID output for distance control"""
    global dist_proportional_gain, dist_integral_gain, dist_derivative_gain
    global dist_accumulated_error, dist_previous_error
    
    control_error = current_dist - target_dist
    dist_accumulated_error = dist_accumulated_error + control_error
    pid_result = (dist_proportional_gain * control_error + 
                 dist_integral_gain * dist_accumulated_error + 
                 dist_derivative_gain * (control_error - dist_previous_error))
    dist_previous_error = control_error
    return pid_result

def compute_angular_pid_control(target_orientation, current_robot_pose):
    """Calculate PID output for angular control"""
    global angular_proportional_gain, angular_integral_gain, angular_derivative_gain
    global angular_accumulated_error, angular_previous_error
    
    normalized_robot_pose = (current_robot_pose / 3.14159265359 * 180.0 + 180.0) % 360
    adjusted_target = target_orientation
    
    if adjusted_target == 0:
        if 0 < normalized_robot_pose < 180:
            adjusted_target = 0
        if 180 < normalized_robot_pose < 360:
            adjusted_target = 360
            
    angular_error = normalized_robot_pose - adjusted_target
    angular_accumulated_error = angular_accumulated_error + angular_error
    pid_output_result = (angular_proportional_gain * angular_error + 
                        angular_integral_gain * angular_accumulated_error + 
                        angular_derivative_gain * (angular_error - angular_previous_error))
    angular_previous_error = angular_error
    return pid_output_result

# ============================================================================
# Robot Movement and Parking Functions
# ============================================================================

def execute_precision_docking_v2(target_x_coord, target_y_coord, target_orientation_angle):
    """High precision parking version 2"""
    global dist_proportional_gain, dist_integral_gain, dist_derivative_gain, dist_accumulated_error
    
    # Reset PID parameters for precision mode
    dist_proportional_gain, dist_integral_gain, dist_derivative_gain = 0.8, 0.001, 0.01
    dist_accumulated_error = 0
    consecutive_success_count = 0
    
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    loop_rate = rospy.Rate(7)
    movement_command = Twist()
    measured_x, measured_y = 0, 0
    iteration_counter = 0
    
    while not rospy.is_shutdown():
        loop_rate.sleep()
        iteration_counter += 1
        
        # X-axis control logic
        if target_x_coord > 0:
            pid_x_output = compute_distance_pid_control(target_x_coord, lidar_front_distance)
            measured_x = lidar_front_distance
        if target_x_coord < 0:
            pid_x_output = compute_distance_pid_control(target_x_coord, -lidar_back_distance)
            measured_x = -lidar_back_distance
            
        # Y-axis control logic
        if target_y_coord > 0:
            pid_y_output = compute_distance_pid_control(target_y_coord, lidar_left_distance)
            measured_y = lidar_left_distance
        if target_y_coord < 0:
            pid_y_output = compute_distance_pid_control(target_y_coord, -lidar_right_distance)
            measured_y = -lidar_right_distance
            
        angular_pid_output = compute_angular_pid_control(target_orientation_angle, robot_orientation_yaw)
        
        movement_command.linear.y = pid_y_output
        movement_command.linear.x = pid_x_output
        movement_command.angular.z = angular_pid_output / 180.0 * 3.14159265359
        
        dist_accumulated_error = constrain_value_within_bounds(dist_accumulated_error, 5)
        
        if iteration_counter >= 150:
            dist_accumulated_error = 0
            break
            
        # Precision check with tight tolerances
        position_x_ok = abs(measured_x - target_x_coord) <= 0.03
        position_y_ok = abs(measured_y - target_y_coord) <= 0.03
        angle_ok = abs(target_orientation_angle - (robot_orientation_yaw / 3.1415926 * 180 + 180)) <= 3
        
        if position_x_ok and position_y_ok and angle_ok:
            dist_accumulated_error = 0
            consecutive_success_count += 1
        
        if consecutive_success_count >= 6:
            movement_command.linear.x = 0
            movement_command.linear.y = 0
            movement_command.linear.z = 0
            velocity_publisher.publish(movement_command)
            dist_accumulated_error = 0
            break
            
        velocity_publisher.publish(movement_command)

def perform_relative_clockwise_90_degree_turn():
    """Execute relative 90-degree clockwise rotation from current angle"""
    cmd_velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rotation_rate = rospy.Rate(10)
    rotation_command = Twist()
    
    # 记录开始角度并计算目标角度
    start_angle = (robot_orientation_yaw / 3.14159265359 * 180.0 + 180.0) % 360
    target_angle = (start_angle - 90) % 360  # 右转90度（顺时针）
    
    print(f"开始角度: {start_angle:.1f}°, 目标角度: {target_angle:.1f}°")
    
    while not rospy.is_shutdown():
        current_robot_angle = (robot_orientation_yaw / 3.14159265359 * 180.0 + 180.0) % 360
        
        # 计算角度差
        angle_difference = (current_robot_angle - target_angle + 540) % 360 - 180
        
        if abs(angle_difference) <= 8:
            rotation_command.angular.z = 0
            cmd_velocity_publisher.publish(rotation_command)
            print(f"转动完成，当前角度: {current_robot_angle:.1f}°")
            break
            
        rotation_command.linear.x = 0
        rotation_command.linear.y = 0
        rotation_command.angular.z = -2.5  # 右转（顺时针）
        
        cmd_velocity_publisher.publish(rotation_command)
        rotation_rate.sleep()

def perform_180_degree_turn():
    """Execute 180-degree turn"""
    cmd_velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rotation_rate = rospy.Rate(10)
    rotation_command = Twist()
    
    # 记录开始角度并计算目标角度
    start_angle = (robot_orientation_yaw / 3.14159265359 * 180.0 + 180.0) % 360
    target_angle = (start_angle + 180) % 360  # 转180度
    
    print(f"开始180度旋转 - 开始角度: {start_angle:.1f}°, 目标角度: {target_angle:.1f}°")
    
    while not rospy.is_shutdown():
        current_robot_angle = (robot_orientation_yaw / 3.14159265359 * 180.0 + 180.0) % 360
        
        # 计算角度差
        angle_difference = (current_robot_angle - target_angle + 540) % 360 - 180
        
        if abs(angle_difference) <= 8:
            rotation_command.angular.z = 0
            cmd_velocity_publisher.publish(rotation_command)
            print(f"180度旋转完成，当前角度: {current_robot_angle:.1f}°")
            break
            
        rotation_command.linear.x = 0
        rotation_command.linear.y = 0
        rotation_command.angular.z = 2.0  # 左转（逆时针）
        
        cmd_velocity_publisher.publish(rotation_command)
        rotation_rate.sleep()

def execute_angle_only_calibration(target_orientation_angle):
    """只校准角度的PID，不调整位置，在当前位置精确调整到目标角度"""
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    loop_rate = rospy.Rate(10)
    movement_command = Twist()
    consecutive_success_count = 0
    iteration_counter = 0
    
    print(f"开始角度校准，目标角度: {target_orientation_angle}°")
    
    while not rospy.is_shutdown():
        loop_rate.sleep()
        iteration_counter += 1
        
        # 只进行角度控制
        angular_pid_output = compute_angular_pid_control(target_orientation_angle, robot_orientation_yaw)
        
        # 不调整位置，只调整角度
        movement_command.linear.x = 0
        movement_command.linear.y = 0
        movement_command.angular.z = angular_pid_output / 180.0 * 3.14159265359
        
        # 超时保护
        if iteration_counter >= 100:
            break
            
        # 角度精度检查
        current_angle_normalized = (robot_orientation_yaw / 3.1415926 * 180 + 180) % 360
        angle_ok = abs(target_orientation_angle - current_angle_normalized) <= 2
        
        if angle_ok:
            consecutive_success_count += 1
        else:
            consecutive_success_count = 0
        
        if consecutive_success_count >= 5:
            movement_command.linear.x = 0
            movement_command.linear.y = 0
            movement_command.angular.z = 0
            velocity_publisher.publish(movement_command)
            print(f"角度校准完成，当前角度: {current_angle_normalized:.1f}°")
            break
            
        velocity_publisher.publish(movement_command)

def execute_velocity_buffer(direction, duration=3.0, speed=2.2):
    """给机器人一个方向速度缓冲"""
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    loop_rate = rospy.Rate(10)
    movement_command = Twist()
    
    if direction == "forward":
        print(f"执行向前速度缓冲，持续{duration}秒...")
        movement_command.linear.x = speed
    elif direction == "backward":
        print(f"执行向后速度缓冲，持续{duration}秒...")
        movement_command.linear.x = -speed
    else:
        print(f"无效的方向: {direction}")
        return
        
    movement_command.linear.y = 0
    movement_command.angular.z = 0
    
    # 执行速度缓冲
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        if (current_time - start_time).to_sec() >= duration:
            break
        velocity_publisher.publish(movement_command)
        loop_rate.sleep()
    
    # 停止运动
    movement_command.linear.x = 0
    velocity_publisher.publish(movement_command)
    print(f"速度缓冲完成")

def execute_additional_mission_after_task_point(final_target):
    """到达任务点后执行额外任务"""
    print(f"\n开始执行任务点后的额外任务，目标值: {final_target}")
    
    if final_target == 1:
        print("执行任务点1后续路径...")
        execute_precision_docking_v2(0.404, 0.472, 180)  # 1角
        print("到达1角点位")
        execute_precision_docking_v2(0.360, -1.802, 180)  # 12中点
        execute_angle_only_calibration(180)
        print("到达12中点")
        # 12中点后给向前速度缓冲
        execute_velocity_buffer("backward")
        execute_precision_docking_v2(-1.850, -1.838, 180)  # 终点
        print("到达终点")
        try:
            playsound("终点.mp3")
            print("播报: 终点")
        except Exception as e:
            print(f"播放失败: 终点.mp3, 错误: {e}")
            
    elif final_target == 2:
        print("执行任务点2后续路径...")
        execute_precision_docking_v2(0.447, -0.356, 180)  # 2角
        print("到达2角点位")
        execute_precision_docking_v2(0.360, -1.802, 180)  # 12中点
        print("到达12中点")
        execute_angle_only_calibration(180)
        # 12中点后给向前速度缓冲
        execute_velocity_buffer("backward")
        execute_precision_docking_v2(-1.850, -1.838, 180)  # 终点
        print("到达终点")
        try:
            playsound("终点.mp3")
            print("播报: 终点")
        except Exception as e:
            print(f"播放失败: 终点.mp3, 错误: {e}")
            
    elif final_target == 3:
        print("执行任务点3后续路径...")
        execute_precision_docking_v2(-0.371, -0.421, 180)  # 3角
        print("到达3角点位")
        execute_precision_docking_v2(-0.400, -1.822, 180)  # 34中点
        print("到达34中点")
        execute_angle_only_calibration(180)
        # 34中点后给向后速度缓冲
        execute_velocity_buffer("forward")
        execute_precision_docking_v2(-1.850, -1.838, 180)  # 终点
        print("到达终点")
        try:
            playsound("终点.mp3")
            print("播报: 终点")
        except Exception as e:
            print(f"播放失败: 终点.mp3, 错误: {e}")
            
    elif final_target == 4:
        print("执行任务点4后续路径...")
        execute_precision_docking_v2(-0.441, 0.375, 180)  # 4角
        print("到达4角点位")
        execute_precision_docking_v2(-0.400, -1.822, 180)  # 34中点
        execute_angle_only_calibration(180)
        print("到达34中点")
        # 34中点后给向后速度缓冲
        execute_velocity_buffer("forward")
        execute_precision_docking_v2(-1.850, -1.838, 180)  # 终点
        print("到达终点")
        try:
            playsound("终点.mp3")
            print("播报: 终点")
        except Exception as e:
            print(f"播放失败: 终点.mp3, 错误: {e}")

def execute_final_mission_based_on_result(base_number, trigger_point):
    """根据第一个点的识别结果和触发点执行最终任务"""
    print(f"\n开始执行最终任务，基准值: {base_number}, 触发点: {trigger_point}")
    
    # 计算最终目标值：如果是5-8则减去4，缩小范围到1-4
    final_target = base_number
    if 5 <= base_number <= 8:
        final_target = base_number - 4
        print(f"基准值 {base_number} 在5-8范围内，缩小后为: {final_target}")
    
    print(f"最终目标值: {final_target}")
    
    if trigger_point == 10:
        # 原来的10点任务路径保持不变
        if final_target == 1:
            print("执行10点触发的任务路径1...")
            execute_precision_docking_v2(0.447, -0.356, 180)
            print("到达第一个中间点")
            execute_precision_docking_v2(0.404, 0.472, 180)
            execute_angle_only_calibration(180)
            print("到达第二个中间点")
            execute_precision_docking_v2(1.162, 1.216, 180)
            print("到达任务点1")
            try:
                playsound("到达任务点1.mp3")
                print("播报: 到达任务点1")
            except Exception as e:
                print(f"播放失败: 到达任务点1.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 2:
            print("执行10点触发的任务路径2...")
            execute_precision_docking_v2(0.447, -0.356, 180)
            print("到达中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(1.182, -1.252, 180)
            print("到达任务点2")
            try:
                playsound("到达任务点2.mp3")
                print("播报: 到达任务点2")
            except Exception as e:
                print(f"播放失败: 到达任务点2.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 3:
            print("执行10点触发的任务路径3...")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(-1.262, -1.242, 180)
            print("到达任务点3")
            try:
                playsound("到达任务点3.mp3")
                print("播报: 到达任务点3")
            except Exception as e:
                print(f"播放失败: 到达任务点3.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 4:
            print("执行10点触发的任务路径4...")
            execute_precision_docking_v2(-0.441, 0.375, 180)
            print("到达第一个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(-1.258, 1.234, 180)
            print("到达任务点4")
            try:
                playsound("到达任务点4.mp3")
                print("播报: 到达任务点4")
            except Exception as e:
                print(f"播放失败: 到达任务点4.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
    elif trigger_point == 11:
        # 11点的任务路径
        if final_target == 1:
            print("执行11点触发的任务路径1...")
            execute_precision_docking_v2(-0.441, 0.375, 180)
            print("到达第一个中间点")
            execute_precision_docking_v2(0.404, 0.472, 180)
            print("到达第二个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(1.162, 1.216, 180)
            print("到达任务点1")
            try:
                playsound("到达任务点1.mp3")
                print("播报: 到达任务点1")
            except Exception as e:
                print(f"播放失败: 到达任务点1.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 2:
            print("执行11点触发的任务路径2...")
            execute_precision_docking_v2(-0.371, -0.421, 180)
            print("到达第一个中间点")
            execute_precision_docking_v2(0.447, -0.356, 180)
            print("到达第二个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(1.182, -1.252, 180)
            print("到达任务点2")
            try:
                playsound("到达任务点2.mp3")
                print("播报: 到达任务点2")
            except Exception as e:
                print(f"播放失败: 到达任务点2.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 3:
            print("执行11点触发的任务路径3...")
            execute_precision_docking_v2(-0.371, -0.421, 180)
            print("到达第一个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(-1.262, -1.242, 180)
            print("到达任务点3")
            try:
                playsound("到达任务点3.mp3")
                print("播报: 到达任务点3")
            except Exception as e:
                print(f"播放失败: 到达任务点3.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 4:
            print("执行11点触发的任务路径4...")
            execute_precision_docking_v2(-0.441, 0.375, 180)
            print("到达第一个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(-1.258, 1.234, 180)
            print("到达任务点4")
            try:
                playsound("到达任务点4.mp3")
                print("播报: 到达任务点4")
            except Exception as e:
                print(f"播放失败: 到达任务点4.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
    elif trigger_point == 12:
        # 12点的任务路径
        if final_target == 1:
            print("执行12点触发的任务路径1...")
            execute_precision_docking_v2(-0.441, 0.375, 180)
            print("到达第一个中间点")
            execute_precision_docking_v2(0.404, 0.472, 180)
            print("到达第二个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(1.162, 1.216, 180)
            print("到达任务点1")
            try:
                playsound("到达任务点1.mp3")
                print("播报: 到达任务点1")
            except Exception as e:
                print(f"播放失败: 到达任务点1.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 2:
            print("执行12点触发的任务路径2...")
            execute_precision_docking_v2(-0.371, -0.421, 180)
            print("到达第一个中间点")
            execute_precision_docking_v2(0.447, -0.356, 180)
            print("到达第二个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(1.182, -1.252, 180)
            print("到达任务点2")
            try:
                playsound("到达任务点2.mp3")
                print("播报: 到达任务点2")
            except Exception as e:
                print(f"播放失败: 到达任务点2.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 3:
            print("执行12点触发的任务路径3...")
            execute_precision_docking_v2(-0.371, -0.421, 180)
            print("到达第一个中间点")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(-1.262, -1.242, 180)
            print("到达任务点3")
            try:
                playsound("到达任务点3.mp3")
                print("播报: 到达任务点3")
            except Exception as e:
                print(f"播放失败: 到达任务点3.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
                
        elif final_target == 4:
            print("执行12点触发的任务路径4...")
            execute_angle_only_calibration(180)
            execute_precision_docking_v2(-1.258, 1.234, 180)
            print("到达任务点4")
            try:
                playsound("到达任务点4.mp3")
                print("播报: 到达任务点4")
            except Exception as e:
                print(f"播放失败: 到达任务点4.mp3, 错误: {e}")
            # 执行任务点后的额外任务
            execute_additional_mission_after_task_point(final_target)
    else:
        print(f"警告: 无效的触发点 {trigger_point} 或目标值 {final_target}, 跳过最终任务")

# ============================================================================
# Sensor Data Processing Functions
# ============================================================================

def find_nearest_valid_laser_measurement(laser_scan, starting_index, search_direction, angular_resolution):
    """Find valid laser distance measurement"""
    total_angles = len(laser_scan.ranges)
    
    for search_offset in range(total_angles):
        current_index = (starting_index + search_offset * search_direction) % total_angles
        if laser_scan.ranges[current_index] != float('inf'):
            raw_distance = laser_scan.ranges[current_index]
            angle_offset = np.radians((current_index - starting_index) * angular_resolution)
            corrected_distance = raw_distance * np.cos(angle_offset)
            return corrected_distance
    return float('inf')

def handle_incoming_laser_scan_data(laser_scan_msg):
    """Process incoming laser scan data"""
    global lidar_front_distance, lidar_back_distance, lidar_left_distance, lidar_right_distance
    global robot_orientation_yaw, complete_laser_scan_data
    
    complete_laser_scan_data = laser_scan_msg.ranges
    front_laser_index, angular_step = 360, 0.5
    
    lidar_front_distance = find_nearest_valid_laser_measurement(laser_scan_msg, front_laser_index, 1, angular_step)
    lidar_back_distance = find_nearest_valid_laser_measurement(laser_scan_msg, 0, 1, angular_step)
    lidar_left_distance = find_nearest_valid_laser_measurement(laser_scan_msg, 540, -1, angular_step)
    lidar_right_distance = find_nearest_valid_laser_measurement(laser_scan_msg, 180, 1, angular_step)

def initialize_laser_data_listener():
    """Start laser scan data subscription"""
    rospy.Subscriber('/scan', LaserScan, handle_incoming_laser_scan_data, queue_size=7)
    rospy.spin()

# ============================================================================
# Vision and Detection Functions
# ============================================================================

#def process_ar_marker_detection(ar_marker_msg):
    #"""Handle AR marker detection messages"""
    #global ar_marker_detection_result
    #if len(ar_marker_msg.markers):
        #ar_marker_detection_result = ar_marker_msg.markers[0].id

#def initialize_ar_detection_listener():
    #"""Start AR marker detection subscription"""
    #rospy.Subscriber('/ar_pose_marker', AlvarMarkers, process_ar_marker_detection, queue_size=10)
    #rospy.spin()

def process_ai_vision_result(ai_vision_msg):
    """Handle AI vision detection messages"""
    global ai_vision_detection_result
    ai_vision_detection_result = int(ai_vision_msg.data)

def initialize_ai_vision_listener():
    """Start AI vision result subscription"""
    rospy.Subscriber('/vision_result', String, process_ai_vision_result, queue_size=20)
    rospy.spin()

def capture_target_detection_with_timeout(maximum_wait_time):
    """Detect target objects with timeout"""
    global ar_marker_detection_result, ai_vision_detection_result
    
    ai_vision_detection_result = 66
    ar_marker_detection_result = 0
    elapsed_time = 0
    detection_rate = rospy.Rate(10)
    rospy.set_param('/top_view_shot_node/im_flag', 1)
    final_detected_object = 88
    
    while not rospy.is_shutdown():
        if ai_vision_detection_result != 66 and not ar_marker_detection_result:
            final_detected_object = ai_vision_detection_result
        else:
            if ar_marker_detection_result == 0:
                final_detected_object = 88
            else:
                final_detected_object = ar_marker_detection_result
                
        elapsed_time += 1
        if elapsed_time >= maximum_wait_time:
            final_detected_object = 999
            break
        if final_detected_object != 88:
            break
        detection_rate.sleep()
        
    return final_detected_object

# ============================================================================
# Navigation Functions
# ============================================================================

def continuously_update_robot_pose():
    """Continuously update robot position and orientation"""
    global robot_pos_x, robot_pos_y, robot_orientation_yaw
    
    pose_update_rate = rospy.Rate(10)
    transform_listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        pose_update_rate.sleep()
        try:
            (translation, rotation) = transform_listener.lookupTransform("map", "base_link", rospy.Time(0))
            robot_pos_x = translation[0]
            robot_pos_y = translation[1]
            euler_angles = tf.transformations.euler_from_quaternion(rotation)
            robot_orientation_yaw = euler_angles[2]
        except Exception as transform_error:
            pass

# ============================================================================
# Mission Execution Functions
# ============================================================================

def perform_initial_navigation_sequence():
    """执行12点巡航，第一个点识别计算式，后续11个点识别并匹配播报，组内识别成功后跳过剩余点"""
    print("=" * 50)
    print("开始12点巡航任务")
    print("=" * 50)
    
    # ========== 第一个点：识别计算式 ==========
    print("\n【第1个点】前往坐标: (0.566, 0.610, 270)")
    execute_precision_docking_v2(0.566, 0.610, 270)
    print("到达第一个点，开始识别计算式...")
    
    # 识别第一个点的计算式
    first_detection = capture_target_detection_with_timeout(150)
    base_number = first_detection
    
    print(f"✓ 识别到计算结果（基准值）: {base_number}")
    
    # 定义线索播报映射
    clue_audio_map = {
        1: "线索葡萄.mp3",
        2: "线索数字五.mp3",
        3: "线索香蕉.mp3",
        4: "线索二维码7.mp3",
        5: "线索苹果.mp3",
        6: "线索二维码8.mp3",
        7: "线索梨.mp3",
        8: "线索数字六.mp3"
    }
    
    # 播报基准值对应的线索
    if base_number in clue_audio_map:
        try:
            playsound(clue_audio_map[base_number])
            print(f"播报线索: {clue_audio_map[base_number]}")
        except Exception as e:
            print(f"播放失败: {clue_audio_map[base_number]}, 错误: {e}")
    
    vocalize_single_number(base_number)
    rospy.sleep(1.0)
    
    # ========== 定义所有点位的完整信息 ==========
    # 格式: (x, y, angle, name, need_turn, group, position_in_group)
    all_points = [
        (0.576, -1.814, 270, "第2个点", False, 1, 1),  # 第2个点，第1条线索
        (0.542, -0.596, 270, "第3个点", False, 1, 2),  # 第3个点，第1条线索
        (0.510, 0.606, 180, "第4个点", True, 2, 1),    # 第4个点，第2条线索
        (0.526, -1.802, 180, "第5个点", False, 2, 2),  # 第5个点，第2条线索
        (0.526, -0.604, 180, "第6个点", False, 2, 3),  # 第6个点，第2条线索
        (0.592, 0.580, 90, "第7个点", True, 3, 1),     # 第7个点，第3条线索
        (0.572, -1.812, 90, "第8个点", False, 3, 2),   # 第8个点，第3条线索
        (0.550, -0.604, 90, "第9个点", False, 3, 3),   # 第9个点，第3条线索
        (0.522, 0.600, 0, "第10个点", True, 4, 1),     # 第10个点，第4条线索
        (0.558, -1.804, 0, "第11个点", False, 4, 2),   # 第11个点，第4条线索
        (0.560, -0.630, 0, "第12个点", False, 4, 3)    # 第12个点，第4条线索
    ]
    
    # 组解锁信息映射
    group_unlock_audio = {
        1: "已解锁第一条线索信息.mp3",
        2: "已解锁第二条线索信息.mp3",
        3: "已解锁第三条线索信息.mp3",
        4: "已解锁第四条线索信息.mp3"
    }
    
    # 定义跳转规则: (当前点编号, 下一个点编号)
    skip_rules = {
        2: 4,   # 第2点识别成功 → 跳到第4点
        3: 4,   # 第3点识别成功 → 跳到第4点
        4: 7,   # 第4点识别成功 → 跳到第7点
        5: 7,   # 第5点识别成功 → 跳到第7点
        6: 7,   # 第6点识别成功 → 跳到第7点
        7: 10,  # 第7点识别成功 → 跳到第10点
        8: 10,  # 第8点识别成功 → 跳到第10点
        9: 10,  # 第9点识别成功 → 跳到第10点
        10: -1, # 第10点识别成功 → 结束
        11: -1, # 第11点识别成功 → 结束
        12: -1  # 第12点识别成功 → 结束
    }
    
    # ========== 遍历所有点位 ==========
    current_index = 0
    skip_turn = False  # 标志位：跳转到达的点位跳过旋转
    task_completed = False  # 标志：任务是否完成
    
    while current_index < len(all_points) and not task_completed:
        x, y, angle, point_name, need_turn, group, pos_in_group = all_points[current_index]
        current_point_number = current_index + 2  # 从第2个点开始
        
        print(f"\n【{point_name}】前往坐标: ({x}, {y}, {angle})")
        
        # 如果不是跳转到达的，正常处理旋转和移动
        if not skip_turn:
            # 如果需要右转90度，先执行右转
            if need_turn:
                print(f"{point_name}需要执行右转90度")
                perform_relative_clockwise_90_degree_turn()
            
            # 到达目标点
            execute_precision_docking_v2(x, y, angle)
            print(f"{point_name}到达，开始识别...")
        else:
            # 跳转到达的点位，已经在正确位置，直接识别
            print(f"{point_name}已通过跳转到达，开始识别...")
            skip_turn = False  # 重置标志位
        
        # 识别当前点
        detection_result = capture_target_detection_with_timeout(150)
        
        # 判断识别结果
        matched = False
        if detection_result != 88 and detection_result != 999:
            # 检查是否为两位数
            if 10 <= detection_result <= 99:
                # 提取十位数字
                first_digit = detection_result // 10
                
                # 判断是否匹配基准值
                if first_digit == base_number:
                    print(f"✓ 识别结果 {detection_result} 匹配基准值 {base_number}，播报")
                    matched = True
                    
                    # 播报"已解锁"信息
                    if group in group_unlock_audio:
                        try:
                            playsound(group_unlock_audio[group])
                            print(f"播报：{group_unlock_audio[group]}")
                        except Exception as e:
                            print(f"播放失败: {group_unlock_audio[group]}, 错误: {e}")
                    
                    # 播报检测结果
                    vocalize_single_number(detection_result)
                    
                    # 根据跳转规则决定下一步
                    if current_point_number in skip_rules:
                        next_point = skip_rules[current_point_number]
                        
                        if next_point == -1:
                            # 任务完成，先执行180度旋转，然后执行新任务
                            print(f"✓ {point_name}识别成功，准备执行180度旋转和后续任务")
                            print("执行180度旋转...")
                            perform_180_degree_turn()
                            print("校准角度到180度...")
                            execute_angle_only_calibration(180)
                            
                            # 执行基于结果的新任务，传递触发点信息
                            execute_final_mission_based_on_result(base_number, current_point_number)
                            
                            task_completed = True
                            break
                        else:
                            # 跳转到指定点
                            print(f"✓ {point_name}识别成功，跳转到第{next_point}点")
                            
                            # 执行跳转移动，移动完成后就直接在目标点位置了
                            if current_point_number == 2:
                                # 第2点识别成功 → 去第3点位置+右转90度+校准角度到180度
                                print("前往第3点位置...")
                                execute_precision_docking_v2(0.542, -0.596, 270)
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到180度...")
                                execute_angle_only_calibration(180)
                                current_index = 2  # 跳到第4点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 3:
                                # 第3点识别成功 → 右转90度+校准角度到180度
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到180度...")
                                execute_angle_only_calibration(180)
                                current_index = 2  # 跳到第4点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 4:
                                # 第4点识别成功 → 去第6点位置+右转90度+校准角度到90度
                                print("前往第6点位置...")
                                execute_precision_docking_v2(0.526, -0.604, 180)
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到90度...")
                                execute_angle_only_calibration(90)
                                current_index = 5  # 跳到第7点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 5:
                                # 第5点识别成功 → 去第6点位置+右转90度+校准角度到90度
                                print("前往第6点位置...")
                                execute_precision_docking_v2(0.526, -0.604, 180)
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到90度...")
                                execute_angle_only_calibration(90)
                                current_index = 5  # 跳到第7点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 6:
                                # 第6点识别成功 → 右转90度+校准角度到90度
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到90度...")
                                execute_angle_only_calibration(90)
                                current_index = 5  # 跳到第7点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 7:
                                # 第7点识别成功 → 去第9点位置+右转90度+校准角度到0度
                                print("前往第9点位置...")
                                execute_precision_docking_v2(0.550, -0.604, 90)
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到0度...")
                                execute_angle_only_calibration(0)
                                current_index = 8  # 跳到第10点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 8:
                                # 第8点识别成功 → 去第9点位置+右转90度+校准角度到0度
                                print("前往第9点位置...")
                                execute_precision_docking_v2(0.550, -0.604, 90)
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到0度...")
                                execute_angle_only_calibration(0)
                                current_index = 8  # 跳到第10点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                                
                            elif current_point_number == 9:
                                # 第9点识别成功 → 右转90度+校准角度到0度
                                print("执行右转90度...")
                                perform_relative_clockwise_90_degree_turn()
                                print("校准角度到0度...")
                                execute_angle_only_calibration(0)
                                current_index = 8  # 跳到第10点索引
                                skip_turn = True  # 标记跳转到达，不需要重复旋转
                                continue
                else:
                    print(f"✗ 识别结果 {detection_result} 不匹配基准值 {base_number}（十位数字为{first_digit}），不播报")
            else:
                print(f"⚠ 识别结果 {detection_result} 不是两位数，不播报")
        else:
            print(f"⚠ {point_name}未检测到有效信息（结果: {detection_result}）")
        
        # 如果没有匹配，继续下一个点
        if not matched:
            print(f"{point_name}未匹配，继续下一个点")
        
        rospy.sleep(0.8)
        current_index += 1
    
    # 如果遍历完所有点位都没有提前结束，也需要执行180度旋转和新任务
    if current_index >= len(all_points) and not task_completed:
        print("所有点位巡航完成，执行最终180度旋转和后续任务...")
        perform_180_degree_turn()
        print("校准角度到180度...")
        execute_angle_only_calibration(180)
        # 传递12作为触发点（假设遍历完所有点位相当于12点触发）
        execute_final_mission_based_on_result(base_number, 12)
    
    print("\n" + "=" * 50)
    print("全部任务完成 ✅")
    print("=" * 50)

# ============================================================================
# System Initialization and Control Functions
# ============================================================================

def initiate_competition_sequence():
    """Start the competition sequence"""
    user_input_value = input("请输入1开始启动: ")
    if user_input_value == "1":
        print("系统启动成功！")
        return True
    else:
        print("输入错误，请重新运行程序")
        return False

def setup_robot_system_components():
    """Initialize all robot system components"""
    global nav_quaternions_list, nav_point_count, move_base_action_client
    
    # Generate quaternions for navigation waypoints only
    for waypoint_idx in range(nav_point_count):
        waypoint_quaternion = tf.transformations.quaternion_from_euler(0, 0, nav_waypoint_coords[waypoint_idx][2])
        nav_quaternions_list.append(waypoint_quaternion)
    
    # Initialize move_base action client
    move_base_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base_action_client.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Request to connect to move_base server")
    rospy.loginfo("Be connected successfully")
    
    # Start all listener threads
    laser_thread = threading.Thread(target=initialize_laser_data_listener)
    laser_thread.start()
    
    #ar_detection_thread = threading.Thread(target=initialize_ar_detection_listener)
    #ar_detection_thread.start()
    
    ai_vision_thread = threading.Thread(target=initialize_ai_vision_listener)
    ai_vision_thread.start()
    
    pose_tracking_thread = threading.Thread(target=continuously_update_robot_pose)
    pose_tracking_thread.start()

# ============================================================================
# Main Program Execution
# ============================================================================

if __name__ == '__main__':
    rospy.init_node('robot_navigation_system', anonymous=True)
    setup_robot_system_components()
    rospy.set_param('/top_view_shot_node/im_flag', 1)
    
    # 等待用户确认启动
    if not initiate_competition_sequence():
        exit()
    
    # Start timing from competition announcement
    mission_start_time = rospy.Time.now()
    playsound("比赛开始.mp3")
    print("比赛开始")
    
    # 执行12点巡航任务（包含识别和播报逻辑）
    perform_initial_navigation_sequence()
    
    # Calculate and display total execution time
    mission_end_time = rospy.Time.now()
    total_execution_seconds = (mission_end_time - mission_start_time).to_sec()
    
    execution_minutes = int(total_execution_seconds // 60)
    execution_seconds = int(total_execution_seconds % 60)
    
    print("=" * 50)
    print("任务执行完成！")
    print(f"总耗时：{execution_minutes}分{execution_seconds}秒")
    print("=" * 50)
