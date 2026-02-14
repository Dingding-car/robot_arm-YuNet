'''
5DoF机械臂的配置文件
----------------------------------------------
@作者: 朱林
@公司: 湖南创乐博智能科技有限公司
@邮箱: zhulin@loborobot.com
@官方网站: https://www.loborobot.com/
'''
import math
import numpy as np
##########################################
## 机械臂-设备
##
##########################################
# 串口舵机的默认设备号

# DEVICE_PORT_DEFAULT = '/dev/ttyAMA0'   # RaspberryPi4
# DEVICE_PORT_DEFAULT = '/dev/ttyUSB0' # RaspberryPi4
# DEVICE_PORT_DEFAULT = '/dev/ttyS0' # RaspberryPi4
DEVICE_PORT_DEFAULT = 'COM8' # PC Windows
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
# JOINT2SERVO_K=[-57.614, 55.768, -57.105, -57.041, -54.558, 59.269]
# JOINT2SERVO_B=[-21.100, -0.200, 29.900, 17.000, -7.600, 1.000]
# JOINT2SERVO_K=[-49.720, -46.473, -57.741, -57.105, -57.487, 58.060]
# JOINT2SERVO_B=[-2.400, -73.300, 29.600, 12.100, -2.100, 0.100]

JOINT2SERVO_K=[-58.505, 57.805, -54.813, -52.330, -57.487, 55.832]
JOINT2SERVO_B=[-1.600, 92.100, -7.900, 5.400, -9.400, 6.700]

# 预设点
# X坐标 Y坐标 Z坐标 俯仰角 横滚角
# [x, y, z, pitch, roll]
# None代表不指定/需要自动生成
# 注: 姿态角为 Z-Y-Z欧拉角
ARM_POSE_DICT = {
    # 零点位置
    # 'home': [150, 0, 200, None, np.pi],
    'home': [168.62, -4, 234.36, 1.489, 3.0],
    # 物块释放点
    'object_release': [0, 185, 80, None, np.pi],
    # 拍摄工作台图像的位姿
    'capture_image': [85, 0.0, 44.0, 2.47, np.pi],  # [105, 0.0, 60.0, 2.47, np.pi]
    # 色块位姿
    'object_green': [120, 130, 8, np.radians(140.0), np.radians(179.2)],
    'object_yellow': [215, 135, 15, np.radians(125), np.radians(179.2)],
    'object_red': [115, -135, 10, np.radians(140), np.radians(179.2)],
    'object_blue': [210, -135, 5, np.radians(135), np.radians(179.2)],
    
    # 松开点位
    'object_green_add': [120, 130, 30, np.radians(140.0), np.radians(179.2)],
    'object_yellow_add': [215, 135, 60, np.radians(125), np.radians(179.2)],
    'object_red_add': [115, -135, 50, np.radians(140), np.radians(179.2)],
    'object_blue_add': [210, -135, 50, np.radians(135), np.radians(179.2)]
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
