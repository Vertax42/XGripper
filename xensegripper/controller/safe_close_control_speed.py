import sys
import time
import numpy as np
import threading
from collections import deque
from dataclasses import dataclass

from xensesdk import Sensor


# 如果是 windows 系统，设置高精度定时器
if sys.platform.startswith("win"):
    import ctypes
    # 设置全局定时器精度为 5ms
    ctypes.windll.winmm.timeBeginPeriod(5)


class SlipDetector:
    """
    基于切向/法向力与其时序特征检测滑移：
    1. 比值阈值: tau = Ft/Fn > mu_warn
    2. 切向力斜率: d(Ft)/dt > dFt_th
    3. 切向力抖动: Ft 在短窗内波动幅度 > jitter_th
    """
    def __init__(self,
                 mu_warn=0.6,          # 摩擦锥预警阈值 (经验/估计的摩擦系数)
                 dFt_th=0.02,          # 切向力斜率阈值 (N/s)
                 jitter_th=0.01,       # 切向力抖动阈值 (N)
                 Fn_pix_th=0.002,      # 判定接触像素的最小法向力 (N)
                 hist_len=10,          # 短窗长度 (取样点数)
                 debounce_k=1,         # 连续K帧满足才判滑
                 eps=1e-6):
        
        self.mu_warn = mu_warn
        self.dFt_th = dFt_th
        self.jitter_th = jitter_th
        self.Fn_pix_th = Fn_pix_th
        self.hist_len = hist_len
        self.debounce_k = debounce_k
        self.eps = eps
        self.buf_Ft = deque(maxlen=hist_len)
        self.buf_tau = deque(maxlen=hist_len)
        self._streak  = 0
    
    def update(self, force, dt):

        Fx = force[:, :, 0]
        Fy = force[:, :, 1]
        Fz = force[:, :, 2]   

        Ft_map = np.sqrt(Fx*Fx + Fy*Fy)
        Fn_map = np.abs(Fz)

        # 自适应接触区域，只看Fn
        mask = Fn_map > self.Fn_pix_th
        if not np.any(mask):
            self._streak = 0
            return False, {"Ft":0.0, "Fn":0.0, "tau":0.0, "dFt":0.0, "jitter":0.0}

        # 用分位数统计接触区内的代表值
        Ft_rep = float(np.percentile(Ft_map[mask], 90))
        Fn_rep = float(np.percentile(Fn_map[mask], 50))
        tau = Ft_rep / max(Fn_rep, self.eps)

        # 短窗feature
        self.buf_Ft.append(Ft_rep)
        self.buf_tau.append(tau)

        if len(self.buf_Ft) >= 2 and dt > 0:
            dFt = (self.buf_Ft[-1] - self.buf_Ft[-2]) / dt
        else:
            dFt = 0.0
        
        # 抖动, 短窗内 max-min
        jitter = (max(self.buf_Ft) - min(self.buf_Ft)) if len(self.buf_Ft) >= self.debounce_k else 0.0

        # 单帧判据, 单帧条件成立并不立即判滑，而是要求连续 debounce_k 帧都成立，才输出 slipping=True
        # cond = (tau > self.mu_warn) or (dFt > self.dFt_th) or (jitter > self.jitter_th)
        cond = (abs(dFt) > abs(self.dFt_th))
        # 需要连续 K 帧为真
        if cond:
            self._streak += 1
        else:
            self._streak = 0

        slipping = (self._streak >= self.debounce_k)

        return slipping, {"Ft":Ft_rep, "Fn":Fn_rep, "tau":tau, "dFt":abs(dFt), "jitter":jitter}
    

@dataclass
class PIDConfig:
    Kp: float
    Ki: float
    Kd: float
    I_limit: float
    dt: float
    output_limit: tuple = (-30, 30)


class PIDController:
    def __init__(self, config: PIDConfig):
        self.set_params(config)
        self.reset()

    def set_params(self, config: PIDConfig):
        self.Kp = config.Kp
        self.Ki = config.Ki
        self.Kd = config.Kd
        self.dt = config.dt
        self.integral_limit = config.I_limit
        self.output_limit = config.output_limit

    def reset(self):
        self.integral = 0
        self.prev_error = 0

    def update(self, target, measured_value):
        error = target - measured_value
        self.integral += error * self.dt
        self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = float(np.clip(output, self.output_limit[0], self.output_limit[1]))
        
        self.prev_error = error
        
        return output

# from xensetool import LinePlotClient
# line = LinePlotClient()

class SafeCtlImpl:
    def __init__(
        self, 
        sensor, 
        gripper,
        pid_config = PIDConfig(Kp=30, Ki=0.5, Kd=0.00, I_limit=10, dt=0.025, output_limit=(-15, 15)),
    ):
        """
        非阻塞安全闭合控制器。
        - step(): 单步执行一次控制（非阻塞）
        - start()/stop(): 以后台线程形式运行
        """
        self.sensor = sensor
        self.gripper_client = gripper
        
        # self.logger = PosLogger()
        self.F_target = 0.35
        self.F_th = 0.05  # 判定接触的力阈值
        
        # 控制参数
        self.pid_ctl = PIDController(pid_config)

        # 线程控制
        self._thread = None
        self._stop_event = threading.Event()

        # 控制量
        self.pos_target = self.gripper_client.buffer.get()["position"]
    
    def set_target_position(self, pos):
        self.pos_target = pos

    def reset(self):
        self.pid_ctl.reset()
        self._stop_event.clear()

    def measure(self):
        force_field, depth = self.sensor.selectSensorInfo(Sensor.OutputType.Force, Sensor.OutputType.Depth)
        depth = depth[10:-10, 10:-10]  # 取第一个通道
        F_now = depth.max()
        pos_now = self.gripper_client.buffer.get()["position"]
        return force_field, float(F_now), pos_now

    def step(self):
        """
        执行一次控制步进（非阻塞）。
        返回字典状态，便于上层记录/显示。
        """
        force_field, F_now, pos_now = self.measure()

        # 张开    
        if self.pos_target > pos_now - 0.08:
            self.gripper_client.set_gripper_position(self.pos_target, 80, 27)
            return

        # 闭合, 未接触: 直接闭合
        if F_now < self.F_th:
            self.gripper_client.set_gripper_speed(self.pid_ctl.output_limit[1], 27)
            return

        # PID 力控
        v_cmd = self.pid_ctl.update(self.F_target, F_now)
        self.gripper_client.set_gripper_speed(v_cmd, 27)
        return

    # ---------- 后台线程模式 ----------
    def _loop(self):
        next_t = time.time()
        while not self._stop_event.is_set():
            
            self.step()
            # 固定周期
            next_t += self.pid_ctl.dt
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                # 落后周期时，尽量追赶但避免忙等
                next_t = time.time()

    def start(self):
        """后台线程方式启动（非阻塞）"""
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, join=True, timeout=None):
        """停止后台线程"""
        self._stop_event.set()
        if join and self._thread:
            self._thread.join(timeout=timeout)
        self.gripper_client.set_gripper_speed(0.0, 27)
