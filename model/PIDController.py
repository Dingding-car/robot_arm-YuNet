import time
import numpy as np

class PIDController2D:
    def __init__(self, kp, ki, kd, min_output, max_output, integral_limit=100):
        """
        二维PID控制器（封装X/Y两个维度的独立PID）
        :param kp: 比例系数（可以是数值或(x,y)元组）
        :param ki: 积分系数（可以是数值或(x,y)元组）
        :param kd: 微分系数（可以是数值或(x,y)元组）
        :param min_output: 输出最小值（角度下限，数值或(x,y)元组）
        :param max_output: 输出最大值（角度上限，数值或(x,y)元组）
        :param integral_limit: 积分限幅（抗积分饱和）
        """
        # 统一处理参数格式：数值→(数值,数值)
        self.kp = (kp, kp) if isinstance(kp, (int, float)) else kp
        self.ki = (ki, ki) if isinstance(ki, (int, float)) else ki
        self.kd = (kd, kd) if isinstance(kd, (int, float)) else kd
        self.min_output = (min_output, min_output) if isinstance(min_output, (int, float)) else min_output
        self.max_output = (max_output, max_output) if isinstance(max_output, (int, float)) else max_output
        self.integral_limit = integral_limit

        # 初始化X/Y维度的PID状态
        self.state = {
            'x': {'last_error': 0.0, 'integral': 0.0, 'last_time': time.time()},
            'y': {'last_error': 0.0, 'integral': 0.0, 'last_time': time.time()}
        }

    def _compute_single(self, dim, setpoint, feedback):
        """内部方法：计算单个维度（x/y）的PID输出"""
        current_time = time.time()
        dt = current_time - self.state[dim]['last_time']
        if dt < 1e-6:
            dt = 1e-6

        # 1. 计算当前误差
        error = setpoint - feedback

        # 2. 比例项
        proportional = self.kp[0 if dim == 'x' else 1] * error

        # 3. 积分项（带限幅）
        self.state[dim]['integral'] += error * dt
        self.state[dim]['integral'] = np.clip(
            self.state[dim]['integral'],
            -self.integral_limit,
            self.integral_limit
        )
        integral = self.ki[0 if dim == 'x' else 1] * self.state[dim]['integral']

        # 4. 微分项
        derivative = self.kd[0 if dim == 'x' else 1] * (error - self.state[dim]['last_error']) / dt
        
        # 5. 更新状态
        self.state[dim]['last_error'] = error
        self.state[dim]['last_time'] = current_time

        # 6. 总输出 + 限幅
        output = proportional + integral + derivative
        output = np.clip(
            output,
            self.min_output[0 if dim == 'x' else 1],
            self.max_output[0 if dim == 'x' else 1]
        )
        return output

    def compute(self, setpoint_2d, feedback_2d):
        """
        计算二维PID输出
        :param setpoint_2d: 二维设定点 (x_set, y_set)
        :param feedback_2d: 二维反馈点 (x_fb, y_fb)
        :return: 二维输出 (x_out, y_out)
        """
        x_out = self._compute_single('x', setpoint_2d[0], feedback_2d[0])
        y_out = self._compute_single('y', setpoint_2d[1], feedback_2d[1])
        return (x_out, y_out)

    def reset(self):
        """重置所有维度的PID状态"""
        for dim in ['x', 'y']:
            self.state[dim]['last_error'] = 0.0
            self.state[dim]['integral'] = 0.0
            self.state[dim]['last_time'] = time.time()