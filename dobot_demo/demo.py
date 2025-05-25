#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 匯入必要的模組
import sys
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import *  # 匯入 Dobot 的服務定義
from std_msgs.msg import Float32MultiArray
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from dobot_demo.model import DeviceDataTypeEnum  # 自訂義資料列舉類別

# 定義機器手臂控制的客戶端 Node
class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 初始化與服務的連線 (建立 client)
        self.EnableRobot_l = self.create_client(EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')
        self.MovJ_l = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')
        self.SpeedFactor_l = self.create_client(SpeedFactor, '/dobot_bringup_ros2/srv/SpeedFactor')
        self.MovL_l = self.create_client(MovL, '/dobot_bringup_ros2/srv/MovL')
        self.DO_l = self.create_client(DO, '/dobot_bringup_ros2/srv/DO')
        self.DI_l = self.create_client(DI, '/dobot_bringup_ros2/srv/DI')
        self.GetAngle_l = self.create_client(GetAngle, '/dobot_bringup_ros2/srv/GetAngle')
        self.GetDOGroup_l = self.create_client(GetDOGroup, '/dobot_bringup_ros2/srv/GetDOGroup')

        # 等待 EnableRobot 服務可用
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    # 初始化機器人（開啟、設定速度、DO 訊號）
    def initialization(self):
        response = self.EnableRobot_l.call_async(EnableRobot.Request())  # 啟用機器手臂 (開啟使能)
        print(response)

        spe = SpeedFactor.Request()
        spe.ratio = 2  # 設定速度比例為 2 倍
        response = self.SpeedFactor_l.call_async(spe)

        self.DO(1, 1)  # DO1 設為高電位（ORG）
        self.DO(3, 1)  # DO3 設為高電位

        print(response)

    # 控制點對點移動，支援 MovJ 與 MovL 兩種模式
    def point(self, Move, X_j1, Y_j2, Z_j3, RX_j4, RY_j5, RZ_j6):
        if Move == "MovJ":
            P1 = MovJ.Request()
        elif Move == "MovL":
            P1 = MovL.Request()
        else:
            print("無該指令")
            return

        # 設定移動參數（6軸位置）
        P1.mode = True
        P1.a = float(X_j1)
        P1.b = float(Y_j2)
        P1.c = float(Z_j3)
        P1.d = float(RX_j4)
        P1.e = float(RY_j5)
        P1.f = float(RZ_j6)

        # 發送請求
        if Move == "MovJ":
            response = self.MovJ_l.call_async(P1)
        else:
            response = self.MovL_l.call_async(P1)

        print(response)

    # 設定數位輸出 DO
    def DO(self, index, status, time=100):
        DO_V = DO.Request()
        DO_V.index = index
        DO_V.status = status
        DO_V.time = time
        response = self.DO_l.call_async(DO_V)
        print(response)

    # 讀取數位輸入 DI
    def DI(self, index):
        DI_V = DI.Request()
        DI_V.index = index

        future = self.DI_l.call_async(DI_V)
        rclpy.spin_until_future_complete(self, future) # 等待 DI 服務回傳資料
        response = future.result()
        return response

    # 取得手臂目前的關節角度
    def GetAngle(self):
        GetAngle_V = GetAngle.Request()
        response = self.GetAngle_l.call_async(GetAngle_V)
        return response

    # 取得 DO 群組狀態
    def GetDOGroup(self):
        GetDOGroup_V = GetDOGroup.Request()
        GetDOGroup_V.index_group = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        response = self.GetDOGroup_l.call_async(GetDOGroup_V)
        return response

    # 發送「啟動」訊號（DO4 = 1）
    def start_point(self):
        self.DO(4, 1)

# 機器手臂監控 Node，接收目標位置並控制移動
class RobotArmMonitor(Node):
    def __init__(self, adder_client):
        super().__init__('robot_arm_monitor')
        self.joint = [0.0] * 6  # 關節初始化
        self.client_node = adder_client
        self.pre_craw = 0  # 前一次的抓取狀態

        # 訂閱來自 DeviceDataTypeEnum.robot_arm 的目標位置
        self.suscriber_Mov = self.create_subscription(
            JointTrajectoryPoint,
            DeviceDataTypeEnum.robot_arm,
            self.listener_callback,
            10
        )

        # 發布實際的手臂關節位置
        self.publisher_joint_position = self.create_publisher(
            Float32MultiArray,
            DeviceDataTypeEnum.realrobot,
            10
        )

        # 每 5 秒執行一次定時回呼
        self.timer = self.create_timer(5.0, self.timer_callback)

    # 接收到新目標後觸發
    def listener_callback(self, msg):
        self.get_logger().info("listener_callback triggered")

        self.joint = msg.positions[0:6]  # 取出前6個作為關節角度
        self.craw = msg.positions[6]    # 第七個為抓取器狀態

        # 以 MovJ 方式移動
        self.client_node.point("MovJ", *self.joint)

        # 抓取動作觸發判斷（從開->閉）
        if self.craw < 0 and self.pre_craw > 0:
            self.client_node.DO(6, 1)  # 抓取 DO6
            self.client_node.start_point()  # 發送啟動信號
            self.pre_craw = self.craw

        # 若抓取器打開，恢復初始狀態
        if self.craw > 0:
            self.client_node.DO(1, 1)  # DO1 ON
            self.pre_craw = self.craw

    # 每 5 秒執行一次的計時器回呼
    def timer_callback(self):
        angles = self.position_returner()  # 取得目前位置
        msg = Float32MultiArray()
        msg.data = angles
        self.publisher_joint_position.publish(msg)  # 發布當前角度
        self.client_node.GetDOGroup()  # 查詢 DO 群組狀態

    # 呼叫 GetAngle 並轉為 float 陣列
    def position_returner(self):
        future = self.client_node.GetAngle()
        rclpy.spin_until_future_complete(self.client_node, future)
        if future.done():
            result = future.result()
            angles = [float(x) for x in result.robot_return.strip('{}').split(',')]
            return angles
        else:
            return []

# 主函數入口
def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS
    node = adderClient('adder_client')  # 建立控制 Node
    node.initialization()               # 初始化控制
    node.GetDOGroup()                   # 查詢 DO 群組狀態
    monitor = RobotArmMonitor(node)     # 建立監控 Node

    rclpy.spin(monitor)  # 執行監控迴圈
    monitor.get_logger().info("Shutting down RobotArmMonitor.")
    node.get_logger().info("Shutting down AdderClient.")
    monitor.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
