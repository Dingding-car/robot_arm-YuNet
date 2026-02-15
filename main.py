import cv2
import sys
sys.path.append('./kinematic')
from kinematic.arm5dof_uservo import Arm5DoFUServo

import threading
import ctypes
import inspect
import keyboard

# //* 舵机串口号
SERVO_PORT = 'COM8'

# 线程结束代码
def _async_raise(tid, exctype):
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")
        
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


# 主函数

def main():
    servo_manager = Arm5DoFUServo(device=SERVO_PORT, is_init_pose= False)
    servo_manager.home()
    # servo_manager.set_damping(1000)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误：无法打开摄像头！")
        return
    
    print("摄像头已打开，按 'q' 键退出")
    
    # 2. 循环读取视频帧并显示
    while True:
        # 读取一帧画面
        # ret: 布尔值，是否成功读取帧
        # frame: 读取到的帧数据（numpy数组格式）
        ret, frame = cap.read()
        
        # 如果读取失败（比如摄像头被断开），退出循环
        if not ret:
            print("错误：无法读取视频帧！")
            break
        
        # 显示帧画面
        # 第一个参数：窗口名称
        # 第二个参数：要显示的帧数据
        cv2.imshow('Camera Video', frame)
        
        # 3. 监听键盘事件，控制退出
        # waitKey(1)：等待1ms获取键盘输入，返回按键的ASCII码
        # ord('q')：获取字符'q'的ASCII码
        # 按q键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 4. 释放资源（必须执行，否则会导致摄像头被占用）
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口


if __name__ == "__main__":
    main()