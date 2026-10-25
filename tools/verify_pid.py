# -*- coding: utf-8 -*-
"""
verify_pid —— PID 阶跃响应仿真（与 algorithm/pid.{h,cpp} 公式严格镜像）
被控对象：一阶电机速度模型  dw/dt = (u - w) / tau   （增益归一化为 1）
用法: python tools/verify_pid.py
展示：P 留稳态误差 -> 加 D 抑制振荡 -> 加 I 消差 -> 积分限幅抗windup
"""
import math

TAU = 0.12      # 电机时间常数 [s]
DT = 0.001      # 控制周期 [s]
T_STEP = 0.1    # 目标阶跃时刻 [s]
T_END = 1.6     # 仿真时长 [s]
TARGET = 10.0   # 阶跃目标（可理解为轮端 rad/s）


class Pid:  # 与 C++ 实现逐行对应
    def __init__(self, kp, ki, kd, i_lim, out_lim):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_lim, self.out_lim = i_lim, out_lim
        self.target = 0.0
        self.i_term = 0.0
        self.prev_fb = None
        self.last_out = 0.0

    def compute(self, fb, dt):
        if dt <= 0:
            return self.last_out
        err = self.target - fb
        self.i_term = max(-self.i_lim, min(self.i_lim, self.i_term + self.ki * err * dt))
        d = 0.0 if self.prev_fb is None else -self.kd * (fb - self.prev_fb) / dt
        self.prev_fb = fb
        out = self.kp * err + self.i_term + d
        self.last_out = max(-self.out_lim, min(self.out_lim, out))
        return self.last_out


def simulate(pid):
    w, log_t, log_w, log_u = 0.0, [], [], []
    n = int(T_END / DT)
    for i in range(n):
        t = i * DT
        pid.target = TARGET if t >= T_STEP else 0.0
        u = pid.compute(w, DT)
        w += DT * (u - w) / TAU
        log_t.append(t); log_w.append(w); log_u.append(u)
    return log_t, log_w, log_u


def metrics(t, w):
    post = [(ti, wi) for ti, wi in zip(t, w) if ti >= T_STEP]
    peak = max(wi for _, wi in post)
    over = (peak - TARGET) / TARGET * 100.0
    settle = None
    for ti, wi in post:
        if abs(wi - TARGET) > 0.05 * TARGET:
            settle = ti
    return peak, over, settle


def ascii_plot(t, w, title):
    W, H, y_min, y_max = 76, 14, 0.0, 13.0
    grid = [[' '] * W for _ in range(H)]
    for c in range(W):                       # 参考线：目标值与 95% 带
        row = int((y_max - TARGET) / (y_max - y_min) * (H - 1))
        grid[row][c] = '-'
    for ti, wi in zip(t, w):
        c = int(ti / T_END * (W - 1))
        r = int((y_max - min(max(wi, y_min), y_max)) / (y_max - y_min) * (H - 1))
        grid[r][c] = '*'
    print(f"\n{title}  (纵轴 0~{y_max}，'-'目标值，'*'响应)")
    for r in range(H):
        print(''.join(grid[r]))


def main():
    cases = [
        ("P    kp=0.8", Pid(0.8, 0, 0, 15, 20)),
        ("PD   kp=2.0 kd=0.08", Pid(2.0, 0, 0.08, 15, 20)),
        ("PID  kp=1.2 ki=8 kd=0.05", Pid(1.2, 8, 0.05, 15, 20)),
        ("饱和windup: ki=15 无积分限幅 输出限12", Pid(1.2, 15, 0.05, 1e9, 12)),
        ("饱和抗饱和: ki=15 积分限幅12 输出限12", Pid(1.2, 15, 0.05, 12, 12)),
    ]
    print("=" * 78)
    print(f"被控对象 dw/dt=(u-w)/tau, tau={TAU}s, 目标阶跃 0->{TARGET} @ {T_STEP}s")
    print("最后两行模拟电机电压受限（输出限幅 12 > 稳态需求 10）时的积分饱和现象")
    print("=" * 78)
    for name, pid in cases:
        t, w, u = simulate(pid)
        peak, over, settle = metrics(t, w)
        settle_s = f"{settle:.2f}s" if settle is not None else f">{T_END}s未整定"
        print(f"{name:<36} 峰值={peak:6.2f}  超调={over:+6.1f}%  "
              f"稳态值={w[-1]:6.2f}  达±5%={settle_s}")
    t, w, _ = simulate(Pid(1.2, 8, 0.05, 15, 20))
    ascii_plot(t, w, "PID kp=1.2 ki=8 kd=0.05 阶跃响应")
    t, w, _ = simulate(Pid(1.2, 15, 0.05, 1e9, 12))
    ascii_plot(t, w, "输出限幅12 + ki=15 无积分限幅：windup 超调")
    print("""
读图结论：
  1) P 控制：稳态值 = kp/(1+kp)·目标 ≈ 4.4，永远到不了 10 —— 比例控制必有稳态误差
  2) PD：响应更快、稳态误差仍在（D 不消差）
  3) PID：I 项把稳态误差积掉，最终到达 10；代价是超调与更长的整定时间
  4) windup 两行：输出限幅 12（>稳态需求 10）时爬升段输出顶格、误差持续为正，
     无积分限幅的积分器越积越多，越过目标后要"还债"，超调大；
     积分限幅 12(>稳态所需10、<无限) 超调显著减小。注意两点：
     - 限幅必须大于稳态所需积分量，否则永远到不了目标（上一版踩的坑）
     - 输出限幅低于目标时（限6那版），系统物理上不可能达标，与 windup 无关
调参口诀（速度环）：先 P 后 D 最后 I；振荡减 P 或加 D；超调减 I""")

if __name__ == "__main__":
    main()
