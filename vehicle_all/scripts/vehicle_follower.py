#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

class VehicleFollower:
    def __init__(self, follower_id, leader_id, desired_distance=0.2):
        """
        车辆跟随节点
        :param follower_id: 跟随车辆的ID (例如: 1, 2, 3)
        :param leader_id: 被跟随车辆的ID (例如: 1, 2, 3)
        :param desired_distance: 期望保持的距离 (米)
        """
        self.follower_id = follower_id
        self.leader_id = leader_id
        self.desired_distance = desired_distance
        
        # 位置信息
        self.leader_pose = None
        self.follower_pose = None
        self.leader_orientation = None
        self.follower_orientation = None
        self.leader_velocity = None  # 前车速度
        
        # PID控制器参数
        self.kp_linear = 1.5  # 线性速度比例系数
        self.kp_angular = 2.0  # 角速度比例系数
        self.max_linear_speed = 1.0  # 最大线速度
        self.max_angular_speed = 1.0  # 最大角速度
        
        # 订阅话题
        leader_odom_topic = "/vehicle_{}/ground_truth/state".format(leader_id)
        follower_odom_topic = "/vehicle_{}/ground_truth/state".format(follower_id)
        
        rospy.Subscriber(leader_odom_topic, Odometry, self.leader_odom_callback)
        rospy.Subscriber(follower_odom_topic, Odometry, self.follower_odom_callback)
        
        # 发布话题
        cmd_vel_topic = "/vehicle_{}/cmd_vel".format(follower_id)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        rospy.loginfo("车辆跟随节点启动: 车辆{}跟随车辆{}，期望距离{}米".format(
            follower_id, leader_id, desired_distance))
    
    def leader_odom_callback(self, msg):
        """前车位置回调"""
        self.leader_pose = msg.pose.pose.position
        self.leader_orientation = msg.pose.pose.orientation
        self.leader_velocity = msg.twist.twist  # 保存前车速度
    
    def follower_odom_callback(self, msg):
        """跟随车辆位置回调"""
        self.follower_pose = msg.pose.pose.position
        self.follower_orientation = msg.pose.pose.orientation
    
    def quaternion_to_yaw(self, quaternion):
        """将四元数转换为yaw角（弧度）"""
        euler = tf.transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler[2]
    
    def calculate_target_position(self):
        """
        计算目标位置：前车位置向后desired_distance米
        """
        if self.leader_pose is None or self.leader_orientation is None:
            return None
        
        # 获取前车的朝向角度
        leader_yaw = self.quaternion_to_yaw(self.leader_orientation)
        
        # 计算目标位置（前车位置向后desired_distance米）
        # 注意：向后是朝向的反方向
        target_x = self.leader_pose.x - self.desired_distance * math.cos(leader_yaw)
        target_y = self.leader_pose.y - self.desired_distance * math.sin(leader_yaw)
        
        return Point(x=target_x, y=target_y, z=self.leader_pose.z)
    
    def calculate_control_command(self):
        """
        计算控制命令
        """
        if self.leader_pose is None or self.follower_pose is None:
            return None
        
        # 创建控制命令
        cmd = Twist()
        
        # 检查前车是否静止（速度阈值）
        velocity_threshold = 0.05  # 速度阈值，小于此值认为静止
        if self.leader_velocity is not None:
            leader_linear_speed = math.sqrt(
                self.leader_velocity.linear.x ** 2 + 
                self.leader_velocity.linear.y ** 2)
            leader_angular_speed = abs(self.leader_velocity.angular.z)
            
            # 如果前车静止，跟随车辆也应该静止
            if leader_linear_speed < velocity_threshold and leader_angular_speed < velocity_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                return cmd
        
        # 计算目标位置
        target = self.calculate_target_position()
        if target is None:
            return None
        
        # 计算当前位置到目标位置的距离和角度
        dx = target.x - self.follower_pose.x
        dy = target.y - self.follower_pose.y
        distance = math.sqrt(dx * dx + dy * dy)
        
        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        
        # 获取当前车辆的朝向
        if self.follower_orientation is None:
            return None
        current_yaw = self.quaternion_to_yaw(self.follower_orientation)
        
        # 计算角度差（归一化到[-pi, pi]）
        angle_error = target_angle - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # PID控制
        # 线性速度：距离越远，速度越大，但不超过最大值
        linear_vel = self.kp_linear * distance
        linear_vel = max(0.0, min(linear_vel, self.max_linear_speed))
        
        # 如果距离很近，降低速度
        if distance < 0.1:
            linear_vel = 0.0
        
        # 角速度：角度差越大，角速度越大
        angular_vel = self.kp_angular * angle_error
        angular_vel = max(-self.max_angular_speed, min(angular_vel, self.max_angular_speed))
        
        # 如果角度差很大，先转向再前进
        if abs(angle_error) > math.pi / 4:  # 45度
            linear_vel *= 0.3  # 降低前进速度
        
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        
        return cmd
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            cmd = self.calculate_control_command()
            if cmd is not None:
                self.cmd_vel_pub.publish(cmd)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('vehicle_follower', anonymous=True)
    
    # 从参数服务器获取配置
    follower_id = rospy.get_param('~follower_id', 2)
    leader_id = rospy.get_param('~leader_id', 1)
    desired_distance = rospy.get_param('~desired_distance', 0.2)
    
    follower = VehicleFollower(follower_id, leader_id, desired_distance)
    
    try:
        follower.run()
    except rospy.ROSInterruptException:
        pass

