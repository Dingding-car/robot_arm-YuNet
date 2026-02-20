import cv2
import numpy as np

import sys
sys.path.append('./kinematic')

from kinematic.arm5dof_uservo import Arm5DoFUServo
from model.yunet import YuNet
from model.PIDController import PIDController2D

import threading
import queue
import time

# //* 舵机串口号
SERVO_PORT = 'COM8'

# 人脸检测可视化函数
def visualize(image, results, box_color=(0, 255, 0), text_color=(0, 0, 255), fps=None):
    output = image.copy()
    landmark_color = [
        (255,   0,   0), # right eye
        (  0,   0, 255), # left eye
        (  0, 255,   0), # nose tip
        (255,   0, 255), # right mouth corner
        (  0, 255, 255)  # left mouth corner
    ]

    if fps is not None:
        cv2.putText(output, 'FPS: {:.2f}'.format(fps), (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color)

    for det in results:
        bbox = det[0:4].astype(np.int32)
        cv2.rectangle(output, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), box_color, 2)

        conf = det[-1]
        cv2.putText(output, '{:.4f}'.format(conf), (bbox[0], bbox[1]+12), cv2.FONT_HERSHEY_DUPLEX, 0.5, text_color)

        landmarks = det[4:14].astype(np.int32).reshape((5,2))
        for idx, landmark in enumerate(landmarks):
            cv2.circle(output, landmark, 2, landmark_color[idx], 2)

    return output

# 检测结果队列（用于主线程和舵机控制线程之间的通信）
servo_queue = queue.Queue(maxsize=1)
def capture_video(camera_id = 0, servo_manager = None):

    # YuNet模型初始化
    model_path = './model/face_detection_yunet_2023mar.onnx'
    model = YuNet(modelPath=model_path,
                  inputSize=[320, 320],
                  confThreshold=0.9,
                  nmsThreshold=0.3,
                  topK=5000,
    )

    deviceId = camera_id # 摄像头设备ID,默认为0
    cap = cv2.VideoCapture(deviceId)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    model.setInputSize([w, h])

    tm = cv2.TickMeter()

    print('Press "q" to quit the demo.')
    while cv2.waitKey(1) != ord('q'):
        hasFrame, frame = cap.read()
        if not hasFrame:
            print('No frames grabbed!')
            break

        # Inference
        tm.start()
        results = model.infer(frame) # results is a tuple shape[N,15]
        tm.stop()
        
        # 向舵机队列放入检测结果（非阻塞，避免阻塞主线程）
        try:
            # 放入结果、画面宽度、高度
            servo_queue.put_nowait((results, w, h))
        except queue.Full:
            # 队列满时移除旧数据，放入新数据
            servo_queue.get_nowait()
            servo_queue.put_nowait((results, w, h))

        frame = visualize(frame, results, fps=tm.getFPS())

        # Visualize results in a new Window
        cv2.imshow('YuNet Demo', frame)

        # servo_control(servo_manager, results, w, h)

        tm.reset()
    
    # 释放资源（必须执行，否则会导致摄像头被占用）
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
    
    # 通知舵机线程停止
    global stop_servo_thread
    stop_servo_thread = True

def servo_control(servo_manager, stop_servo_thread= False):

    # PID参数设置
    Kp = (0.05, 0.2)
    Ki = (0.015, 0.01)
    Kd = (0.06, 0.05)

    # 初始化二维PID控制器
    # 可针对X/Y设置不同参数：比如水平响应快一点，垂直稳一点
    pid_2d = PIDController2D(
        kp=Kp,
        ki=Ki,
        kd=Kd,
        min_output=(-90, -90),  # X/Y输出下限
        max_output=(90, 90),    # X/Y输出上限
        integral_limit=50       # 积分限幅
    )

    servo_raw_angle = servo_manager.get_servo_angle_list()
    pan_angle = servo_raw_angle[0]  # 初始化云台水平角度
    tilt_angle = servo_raw_angle[2] # 初始化云台垂直角度

    while not stop_servo_thread:
        try:
            # 非阻塞获取队列数据
            results, dispW, dispH = servo_queue.get_nowait()
        except queue.Empty:
            time.sleep(0.01)
            continue

        if servo_manager is None or len(results) <= 0:
            pid_2d.reset()
            continue
        
        
        face = results[0]
        # 鼻尖坐标
        nose = (face[8], face[9])   # (nose_x, nose_y)

        # 设定点
        setpoint = (dispW // 2, dispH // 2)

        # PID输出
        pid_output = pid_2d.compute(nose, setpoint)

        pan_angle = pid_output[0]
        tilt_angle = -pid_output[1]

        # 最终角度限位（双重保险）
        pan_angle = np.clip(pan_angle, -90, 90)
        tilt_angle = np.clip(tilt_angle, -90, 90)

        # 执行舵机控制
        try:
            servo_manager.uservo.set_servo_angle(0, pan_angle)
            servo_manager.uservo.set_servo_angle(2, tilt_angle)
            print(f"二维PID输出 | 水平角度: {pan_angle:.1f}° | 垂直角度: {tilt_angle:.1f}° | "
                  f"设定点: {setpoint} | 反馈点: {nose}")
        except Exception as e:
            print(f"舵机控制异常: {e}")
            continue

        # # 计算误差
        # error_x = nose_x - dispW // 2
        # error_y = nose_y - dispH // 2

        # # 计算新的舵机角度
        # if abs(error_x) > 20:
        #     pan_angle += Kp * error_x
        # if abs(error_y) > 20:
        #     tilt_angle -= Kp * error_y

        # # 限制角度范围
        # pan_angle = max(-90, min(90, pan_angle))
        # tilt_angle = max(-90, min(90, tilt_angle))

        # # 执行舵机控制（耗时操作，在独立线程中执行）
        # servo_manager.uservo.set_servo_angle(0, pan_angle)
        # servo_manager.uservo.set_servo_angle(2, tilt_angle)

# 主函数
def main():
    servo_manager = Arm5DoFUServo(device=SERVO_PORT, is_init_pose= False)
    servo_manager.home()
    # servo_manager.set_damping(1000)

    # 启动舵机控制线程
    servo_thread = threading.Thread(target=servo_control, args=(servo_manager,), daemon=True)
    servo_thread.start()
    print("舵机控制线程已启动")

    capture_video(camera_id=0)

    # 等待舵机线程结束
    servo_thread.join(timeout=1)
    print("舵机正常退出")
    # 舵机归位（可选） 
    servo_manager.home()
    servo_manager.set_damping(1000)



if __name__ == "__main__":
    main()