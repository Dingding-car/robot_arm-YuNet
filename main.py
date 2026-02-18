import cv2
import numpy as np
import sys
sys.path.append('./kinematic')
from kinematic.arm5dof_uservo import Arm5DoFUServo

from model.yunet import YuNet
import threading
import ctypes
import inspect
import keyboard

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


def capture_video():

    # YuNet模型初始化
    model_path = './model/face_detection_yunet_2023mar.onnx'
    model = YuNet(modelPath=model_path,
                  inputSize=[320, 320],
                  confThreshold=0.9,
                  nmsThreshold=0.3,
                  topK=5000,
    )

    deviceId = 0 # 摄像头设备ID
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
        results = model.infer(frame) # results is a tuple
        tm.stop()

        # Draw results on the input image
        frame = visualize(frame, results, fps=tm.getFPS())

        # Visualize results in a new Window
        cv2.imshow('YuNet Demo', frame)

        tm.reset()
    
    # 释放资源（必须执行，否则会导致摄像头被占用）
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口

# 主函数
def main():
    servo_manager = Arm5DoFUServo(device=SERVO_PORT, is_init_pose= False)
    servo_manager.home()
    servo_manager.set_damping(1000)
    capture_video()


if __name__ == "__main__":
    main()