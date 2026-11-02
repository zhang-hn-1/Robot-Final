# =====================================================================
# 【原始文件｜仅追加本注释头，正文与原作者中文注释一字未改】
# 文件名 : config.py（5DoF 机械臂配置文件）
# 来源   : DICE SDK（https://www.dice.com.cn/），取自 src_ext/lab6/config.py
#          （与 color_track/config.py MD5 完全一致 = 47cbb1fd9ec31f9915b2aa7833fdab5f）
# 说明   : JOINT2SERVO_K / JOINT2SERVO_B 文件中以注释形式保留了 4 组历史标定值，
#          末行（未注释）为最终采用值，与本项目实验报告引用的参数一致。
#          交付材料中 lab3/实验3、lab4/实验4 另有一版更早的标定（K 第 1 项 -57.678），
#          本包只收最终版，未收录旧版，特此说明。
# =====================================================================
'''
5DoF机械臂的配置文件
----------------------------------------------
@https://www.dice.com.cn/
'''
import math
import numpy as np
##########################################
## 机械臂-设备
##
##########################################
# 串口舵机的默认设备号

DEVICE_PORT_DEFAULT = '/dev/ttyTHS1'   # Jetson Nano 
# DEVICE_PORT_DEFAULT = '/dev/ttyUSB0' # RaspberryPi4
# DEVICE_PORT_DEFAULT = '/dev/ttyS0' # RaspberryPi4
# DEVICE_PORT_DEFAULT = 'COM6' # PC Windows
# DEVICE_PORT_DEFAULT = '/dev/ttyUSB0' # PC Ubuntu

##########################################
## 串口舵机参数
##
##########################################
SERVO_NUM = 6 # 舵机的个数
SERVO_SPEED_DEFAULT = 100 # 舵机默认的平均转速 °/s
SERVO_SPEED_MIN = 1 # 转速的最大值
SERVO_SPEED_MAX = 200 # 转速的最小值

##########################################
## 机械臂-关节设置
##
##########################################
# 关节名称与舵机ID的映射字典
# 注: 关节名称跟URDF模型中的定义保持一致
JOINT_SERVO_ID_DICT = {
    "joint1": 0,
    "joint2": 1,
    "joint3": 2,
    "joint4": 3,
    "joint5": 4,
    "left_gripper_joint": 5
}

# 舵机原始角度与关节弧度转换对应的偏移量与比例系数
#JOINT2SERVO_K=[-57.614, 56.850, -57.805, -56.850, -58.951, 58.951]
#JOINT2SERVO_B=[-1.500, 78.400, -3.600, 1.800, 0.000, -0.500]
#JOINT2SERVO_K=[-56.850, 59.842, -56.341, -57.678, 0.064, 56.659]
#JOINT2SERVO_B=[-0.100, 90.700, -0.400, 0.000, -0.100, 6.100]
#JOINT2SERVO_K=[-57.678, 59.015, -59.333, -55.513, -54.558, 59.460]
#JOINT2SERVO_B=[1.600, 86.600, 7.400, 8.900, -3.600, 0.600]
#JOINT2SERVO_K=[-59.078, 58.060, -57.614, -58.760, -58.887, 58.505]
#JOINT2SERVO_B=[3.300, 87.700, 6.200, 5.200, -0.400, 0.800]
JOINT2SERVO_K=[-59.142, 57.678, -52.075, -55.513, 0.318, 57.296]
JOINT2SERVO_B=[-0.400, 92.900, 2.000, 3.200, 1.300, 1.600]

# 预设点
# X坐标 Y坐标 Z坐标 俯仰角 横滚角
# [x, y, z, pitch, roll]
# None代表不指定/需要自动生成
# 注: 姿态角为 Z-Y-Z欧拉角
ARM_POSE_DICT = {
    # 零点位置
    'home': [150, 0, 200, None, np.pi],
    # 物块释放点
    'object_release': [0, 185, 80, None, np.pi],
    # 拍摄工作台图像的位姿
    'capture_image': [105, 0.0, 60.0, 2.47, np.pi], 
    # 随机测试位姿
    'object_green': [120, 134, 20, np.radians(140.0), np.radians(179.2)],
    'object_yellow': [220, 135, 30, np.radians(125), np.radians(179.2)],
    'object_red': [110, -136, 20, np.radians(140), np.radians(179.2)],
    'object_blue': [205, -145, 30, np.radians(135), np.radians(179.2)]
}

# Pitch平面拟合结果
PITCH_PANEL_A = -0.002490
PITCH_PANEL_B = -0.004985
PITCH_PANEL_C = 3.048636


##########################################
## 轨迹规划
##
##########################################
TRAJECTORY_DELTA_T = 0.012 # 单位s

##########################################
## 插补算法
##
##########################################
RAD_PER_STEP = 0.05
