#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import *
from std_msgs.msg import Float32MultiArray
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from dobot_demo.model import DeviceDataTypeEnum

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self.EnableRobot_l = self.create_client(EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')
        self.MovJ_l = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')
        self.SpeedFactor_l = self.create_client(SpeedFactor, '/dobot_bringup_ros2/srv/SpeedFactor')
        self.MovL_l = self.create_client(MovL, '/dobot_bringup_ros2/srv/MovL')
        self.DO_l = self.create_client(DO, '/dobot_bringup_ros2/srv/DO')
        self.DI_l = self.create_client(DI, '/dobot_bringup_ros2/srv/DI')
        self.GetAngle_l = self.create_client(GetAngle, '/dobot_bringup_ros2/srv/GetAngle')
        self.GetDOGroup_l = self.create_client(GetDOGroup, '/dobot_bringup_ros2/srv/GetDOGroup')
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def initialization(self):
        response = self.EnableRobot_l.call_async(EnableRobot.Request())
        print(response)
        spe = SpeedFactor.Request()
        spe.ratio = 2
        response = self.SpeedFactor_l.call_async(spe)
        self.DO(1, 1)  # DO1 = ORG    
        self.DO(3, 1)
 
        print(response)

    def point(self, Move, X_j1, Y_j2, Z_j3, RX_j4, RY_j5, RZ_j6):
        if Move == "MovJ":
            P1 = MovJ.Request()
            P1.mode = True
            P1.a = float(X_j1)
            P1.b = float(Y_j2)
            P1.c = float(Z_j3)
            P1.d = float(RX_j4)
            P1.e = float(RY_j5)
            P1.f = float(RZ_j6)
            response = self.MovJ_l.call_async(P1)
            print(response)
        elif Move == "MovL":
            P1 = MovL.Request()
            P1.mode = True
            P1.a = float(X_j1)
            P1.b = float(Y_j2)
            P1.c = float(Z_j3)
            P1.d = float(RX_j4)
            P1.e = float(RY_j5)
            P1.f = float(RZ_j6)
            response = self.MovL_l.call_async(P1)
            print(response)
        else:
            print("無該指令")

    def DO(self, index, status,time=100):
        DO_V = DO.Request()
        DO_V.index = index
        DO_V.status = status
        DO_V.time=time
        response = self.DO_l.call_async(DO_V)
        print(response)
    def DI(self,index):
        DI_V=DI.Request()
        DI_V.index=index

        future = self.DI_l.call_async(DI_V)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response

    def GetAngle(self):
        GetAngle_V = GetAngle.Request()
        response = self.GetAngle_l.call_async(GetAngle_V)
        return response
    def GetDOGroup(self):
        GetDOGroup_V=GetDOGroup.Request()
        GetDOGroup_V.index_group = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        response=self.GetDOGroup_l.call_async(GetDOGroup_V)
        return response
    def start_point(self):
        # 啟動 START 訊號
        self.DO(4, 1)
class RobotArmMonitor(Node):
    def __init__(self, adder_client):
        super().__init__('robot_arm_monitor')
        self.joint = [0.0] * 6
        self.client_node = adder_client
        self.pre_craw = 0
        self.suscriber_Mov = self.create_subscription(
            JointTrajectoryPoint,
            DeviceDataTypeEnum.robot_arm,
            self.listener_callback,
            10
        )

        self.publisher_joint_position = self.create_publisher(
            Float32MultiArray,  #
            DeviceDataTypeEnum.realrobot,     
            10
        )
        # Set up a timer to publish joint angles and call GetDOGroup every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info("listener_callback triggered")
        self.joint = msg.positions[0:6]
        self.craw=msg.positions[6]
        self.client_node.point("MovJ", *self.joint)
        
        if(self.craw<0 and self.pre_craw>0) :
            self.client_node.DO(6,1)
            self.client_node.start_point()
            self.pre_craw = self.craw
        
        if(self.craw>0) :
            self.client_node.DO(1, 1)
            self.pre_craw = self.craw

    def timer_callback(self):
        angles = self.position_returner()
        # ✅ 發佈 joint angles
        msg = Float32MultiArray()
        msg.data = angles
        self.publisher_joint_position.publish(msg)
        self.client_node.GetDOGroup()

    def position_returner(self):
        future = self.client_node.GetAngle()
        rclpy.spin_until_future_complete(self.client_node, future)
        if future.done():
            result = future.result()
            angles = [float(x) for x in result.robot_return.strip('{}').split(',')]
            return angles
        else:
            return []

def main(args=None):
    rclpy.init(args=args)
    node = adderClient('adder_client')
    node.initialization()
    node.GetDOGroup()
    monitor = RobotArmMonitor(node)

    rclpy.spin(monitor)
    monitor.get_logger().info("Shutting down RobotArmMonitor.")
    node.get_logger().info("Shutting down AdderClient.")
    monitor.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
