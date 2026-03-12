import time
import numpy as np
import threading
from xensesdk import Sensor
import numpy as np
import time
import threading
from collections import deque

import time
import matplotlib.pyplot as plt

class PosLogger:
    def __init__(self):
        self.positions = []
        self.timestamps = []
        self.t0 = None

    def log(self, pos, t=None):
        """记录一次 pos 和时间戳"""
        if t is None:
            t = time.time()
        if self.t0 is None:
            self.t0 = t
        self.positions.append(pos)
        self.timestamps.append(t)

    def plot(self):
        """画出 pos 随时间的曲线"""
        times = [ts - self.t0 for ts in self.timestamps]

        # 如果 pos 是标量，直接画一条线
        if isinstance(self.positions[0], (int, float)):
            plt.plot(times, self.positions, marker="o")
            plt.ylabel("Position")
        else:
            # 如果 pos 是向量（比如 [x,y,z]），分别画多条线
            n = len(self.positions[0])
            for i in range(n):
                plt.plot(times, [p[i] for p in self.positions], marker="o", label=f"pos[{i}]")
            plt.legend()
            plt.ylabel("Position components")

        plt.xlabel("Time (s)")
        plt.title("Position vs Time")
        plt.grid(True)
        plt.show()

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
    
class AdaptiveGrip:
    def __init__(self, F_des_init=0.03, F_des_max=0.08):
        self.F_des_init = F_des_init
        self.F_des      = F_des_init
        self.F_des_max  = F_des_max

    def update(self, slipping, now, dt):

        if slipping is True:
            self.F_des = self.F_des_max
        else:
            self.F_des = self.F_des_init

        return self.F_des
    
class SafeCloseController:
    def __init__(self, sensor, gripper,
                 F_des_init=0.03, F_max=0.08, F_th=0.02,
                 dt=0.025, v_pre=15, v_max=15, v_min=0,
                 Kp=1, Ki=0.001, Kd=0.01, I_limit=1,
                 pos_init=70, pos_min=0, pos_max=85,
                 slip_cfg=dict(mu_warn=0.6, dFt_th=0.01, jitter_th=0.01,
                               Fn_pix_th=0.002, hist_len=8, debounce_k=1),
                 on_log=None):
        """
        非阻塞安全闭合控制器。
        - step(): 单步执行一次控制（非阻塞）
        - start()/stop(): 以后台线程形式运行
        """
        self.sensor = sensor
        self.gripper_client = gripper
        self.logger = PosLogger()

        # 控制参数
        self.F_des_init = F_des_init
        self.F_max = F_max
        self.F_th = F_th
        self.dt = dt
        self.v_pre = v_pre
        self.v_max = v_max
        self.v_min = v_min

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.I_limit = I_limit

        self.pos = float(pos_init)
        self.pos_min = pos_min
        self.pos_max = pos_max

        # 状态
        self.integral_e = 0.0
        self.prev_e = 0.0
        self.prev_time = None

        # 组件
        self.slip_det = SlipDetector(**slip_cfg)
        self.F_sched = AdaptiveGrip(F_des_init=F_des_init, F_des_max=F_max)

        # 线程控制
        self._thread = None
        self._stop_evt = threading.Event()

        # 日志回调，可传入函数接收每步的指标/状态
        self.on_log = on_log

        # 控制量
        self.target_position = 0
    
    def set_target_position(self, pos):
        self.target_position = pos

    def reset(self, pos=None):
        self.integral_e = 0.0
        self.prev_e = 0.0
        if pos is not None:
            self.pos = float(pos)
        self.prev_time = None
        self._stop_evt.clear()

    def _read_force(self):
        force = self.sensor.selectSensorInfo(Sensor.OutputType.Force)
        F_now = np.linalg.norm(force, axis=2).max()
        return force, float(F_now)

    def step(self):
        """
        执行一次控制步进（非阻塞）。
        返回字典状态，便于上层记录/显示。
        """
        dt = self.dt

        force, F_now = self._read_force()

        curr_position = self.gripper_client.buffer.get()["position"]
        holding = (abs(self.target_position-curr_position) < 0.08)
        if holding:
            info = {
                "phase": "holding",
                "F_now": F_now,
                "pos": curr_position,
                "v_cmd": 0.0,
                "slip": False,
            }
            if self.on_log: 
                self.on_log(info)
            return info
        
        if self.target_position >= curr_position:
            self.pos = self.target_position
            self.gripper_client.set_gripper_position(self.pos, 80, 27)
            info = {
                "phase": "approach",
                "F_now": F_now,
                "pos": self.pos,
                "v_cmd": self.v_pre,  # 这里是“闭合速度标称值”
                "slip": False,
            }
            if self.on_log: 
                self.on_log(info)
            return info

        # (1) 未接触: 直接闭合
        if F_now < self.F_th:
            self.pos -= self.v_pre * dt
            self.pos = float(np.clip(self.pos, self.pos_min, self.pos_max))
            
            self.gripper_client.set_gripper_position(self.pos, 80, 27)
            
            info = {
                "phase": "approach",
                "F_now": F_now,
                "pos": self.pos,
                "v_cmd": self.v_pre,  # 这里是“闭合速度标称值”
                "slip": False,
            }
            if self.on_log: 
                self.on_log(info)
            return info

        # (2) 已接触: 滑移检测 + 期望力调度
        slipping, metrics = self.slip_det.update(force, dt)
        F_des = self.F_sched.update(slipping, time.time(), dt)

        # PID 力控
        e = F_des - F_now
        self.integral_e += e * dt
        self.integral_e = float(np.clip(self.integral_e, -self.I_limit, self.I_limit))
        derivative = (e - self.prev_e) / dt
        v_cmd = self.Kp * e + self.Ki * self.integral_e + self.Kd * derivative
        v_cmd = float(np.clip(v_cmd, self.v_min, self.v_max))

        # 用“微位移”实现速度效果
        self.pos -= v_cmd * dt
        self.pos = float(np.clip(self.pos, self.pos_min, self.pos_max))
        curr_position = self.gripper_client.buffer.get()["position"]
        if self.target_position >= curr_position:
            self.pos = curr_position

        self.gripper_client.set_gripper_position(self.pos, 80, 27)
        # (3) 结束/保持判据（位置保持由上层决定是否停止）
        holding = (abs(e) < 0.005) or (F_now >= self.F_max)
        # if holding:
        #     self.gripper_client.set_position(self.pos)

        self.prev_e = e

        info = {
            "phase": "force_ctrl",
            "F_now": F_now,
            "F_des": F_des,
            "pos": self.pos,
            "v_cmd": v_cmd,
            "e": e,
            "holding": holding,
            "slip": bool(slipping),
            **{f"m_{k}": v for k, v in metrics.items()}
        }
        if self.on_log:
            self.on_log(info)
        return info

    # ---------- 后台线程模式 ----------
    def _loop(self):
        next_t = time.time()
        while not self._stop_evt.is_set():
            
            info = self.step()
            info["pos"]
            start = time.time()
            self.logger.log(info["pos"])
            # 固定周期
            next_t += self.dt
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
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, join=True, timeout=None):
        """停止后台线程"""
        self._stop_evt.set()
        if join and self._thread:
            self._thread.join(timeout=timeout)
        self.logger.plot()

    def set_target_position(self, pos):
        """
        设置目标位置，控制器会在下一个 step() 中应用。
        """
        self.target_position = pos

