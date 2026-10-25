# -*- coding: utf-8 -*-
"""
verify_integration —— M0 全链路假输入/假输出自测
镜像链路：Dr16 帧计数 -> SafetyGate 状态机 -> 模式映射 -> Chassis(旋转+解算+PID)
与 C++ 实现（module/safety.cpp, module/chassis.cpp, algorithm/pid.cpp）公式一致。
七个阶段覆盖 task.md 安全门槛全部条款。用法: python tools/verify_integration.py
"""
import math

# ---------- 配置（与 bsp/config.h 一致） ----------
BOOT_DELAY, RC_TIMEOUT = 2.0, 0.100
MAX_VX, MAX_VY, MAX_WZ = 2.0, 2.0, 3.0
SPIN_OMEGA = 0.2
R, A, B, K = 0.07786, 0.275, 0.225, 0.5

WHEEL_PID = dict(kp=8.0, ki=0.5, kd=0.0, i_lim=8000.0, out_lim=16384.0)
FOLLOW_PID = dict(kp=4.0, ki=0.0, kd=0.1, i_lim=1.5, out_lim=MAX_WZ)


class Pid:
    def __init__(self, p):
        self.p = p; self.target = 0.0
        self.i = 0.0; self.prev = None; self.out = 0.0
    def compute(self, fb, dt):
        e = self.target - fb
        self.i = max(-self.p["i_lim"], min(self.p["i_lim"], self.i + self.p["ki"] * e * dt))
        d = 0.0 if self.prev is None else -self.p["kd"] * (fb - self.prev) / dt
        self.prev = fb
        self.out = max(-self.p["out_lim"], min(self.p["out_lim"], self.p["kp"] * e + self.i + d))
        return self.out
    def reset(self):
        self.i = 0.0; self.prev = None; self.out = 0.0


class SafetyGate:  # 镜像 module/safety.cpp
    BOOT, WAIT, OUT, STOP = "BOOT_DELAY", "WAIT_RC", "OUTPUT_ALLOWED", "SAFE_STOP"
    def __init__(self, boot_s=BOOT_DELAY):
        self.boot_s = boot_s; self.state = self.BOOT
        self.last_cnt, self.last_t = 0, -1.0
    def rc_alive(self, t, cnt):
        if cnt != self.last_cnt:
            self.last_cnt, self.last_t = cnt, t
            return True
        return self.last_t >= 0 and (t - self.last_t) <= RC_TIMEOUT
    def update(self, t, cnt, estop, centered, link_ok=True):
        alive = self.rc_alive(t, cnt)
        if self.state == self.BOOT:
            if t < self.boot_s:
                return self.state
            self.state = self.WAIT
        if self.state == self.WAIT:
            if alive and not estop and link_ok:
                self.state = self.OUT
            return self.state
        if self.state == self.OUT:
            if not alive or estop or not link_ok:
                self.state = self.STOP
            return self.state
        # SAFE_STOP：人工复位 = 故障源清除 + 摇杆回中
        if alive and not estop and link_ok and centered:
            self.state = self.WAIT
        return self.state


class Chassis:  # 镜像 module/chassis.cpp
    def __init__(self):
        self.mode = "separate"
        self.wp = [Pid(WHEEL_PID) for _ in range(4)]
        self.fp = Pid(FOLLOW_PID)
    def set_mode(self, m):
        if m != self.mode:
            self.mode = m
            for p in self.wp: p.reset()
            self.fp.reset()
    def update(self, rc, theta, dt):
        vg = dict(vx=rc["lv"] * MAX_VX, vy=rc["lh"] * MAX_VY, wz=rc["wheel"] * MAX_WZ)
        if self.mode == "follow":
            self.fp.target = 0.0
            vg["wz"] += self.fp.compute(-wrap_pi(theta), dt)
        elif self.mode == "spin":
            vg["wz"] += SPIN_OMEGA
        c, s = math.cos(theta), math.sin(theta)
        vx, vy = c * vg["vx"] - s * vg["vy"], s * vg["vx"] + c * vg["vy"]
        wz = vg["wz"]
        tgt = [(vx - vy - K * wz) / R, (vx + vy + K * wz) / R,
               (vx + vy - K * wz) / R, (vx - vy + K * wz) / R]
        cmd = []
        for i in range(4):
            self.wp[i].target = tgt[i]
            cmd.append(self.wp[i].compute(0.0, dt))  # 反馈占位 0（M2 接真实值）
        return tgt, cmd, wz


def wrap_pi(a):
    while a > math.pi: a -= 2 * math.pi
    while a <= -math.pi: a += 2 * math.pi
    return a


CENTER = dict(lh=0.0, lv=0.0, rh=0.0, rv=0.0, wheel=0.0)
results = []
def check(name, cond, detail=""):
    results.append((name, cond, detail))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


def main():
    dt, T = 0.002, 0.0
    gate, ch = SafetyGate(), Chassis()
    cnt = 0
    frame_period, t_frame = 0.007, 0.0  # DBus 7ms 一帧（协议 V1.4）
    rc_on, estop, sticks = True, False, CENTER
    theta, mode_req = 0.0, "separate"

    def step(t):
        nonlocal cnt
        if rc_on and t - t_frame >= frame_period >= 0:
            cnt += 1
        st = gate.update(t, cnt, estop, all(abs(v) < 0.05 for v in sticks.values()))
        ch.set_mode(mode_req)
        if st == gate.OUT:
            tgt, cmd, wz = ch.update(sticks, theta, dt)
            return st, tgt, cmd, wz
        for p in ch.wp: p.reset()  # 主循环安全包装：非输出态清零+复位
        return st, [0.0] * 4, [0.0] * 4, 0.0

    # ---- 阶段1：上电 0~2s，遥控器已开 ----
    while T < 1.5:
        _, tgt, cmd, _ = step(T); T += dt
    check("1 上电延时内不输出", gate.state == gate.BOOT and all(c == 0 for c in cmd))

    # ---- 阶段2：2s 后自动进入输出态，分离模式 θ=0 前进 ----
    theta, sticks = 0.0, dict(CENTER, lv=0.5)
    while T < 2.5:
        st, tgt, cmd, _ = step(T); T += dt
    check("2a 启动后放行输出", st == gate.OUT)
    check("2b θ=0 前进指令 -> 四轮同向正转",
          tgt[0] > 0 and abs(tgt[0] - tgt[1]) < 1e-6 and abs(tgt[0] - tgt[2]) < 1e-6,
          f"tgt={[f'{x:.2f}' for x in tgt]}")

    # ---- 阶段3：θ=+90°，杆仍向前 -> 车左移（vy>0 轮型） ----
    theta = math.pi / 2
    for _ in range(10):
        st, tgt, cmd, _ = step(T); T += dt
    check("3 θ=+90° 前推杆 -> 底盘左移", tgt[0] < 0 < tgt[1] and tgt[2] > 0 > tgt[3],
          f"tgt={[f'{x:.2f}' for x in tgt]}（左前- 右前+ 左后+ 右后-）")

    # ---- 阶段4：小陀螺 ----
    mode_req = "spin"
    for _ in range(10):
        st, tgt, cmd, wz = step(T); T += dt
    check("4 小陀螺叠加 0.2rad/s", abs(wz - SPIN_OMEGA) < 1e-6 and st == gate.OUT)

    # ---- 阶段5：跟随模式 θ=0.5 -> wz>0 追赶 ----
    mode_req, theta, sticks = "follow", 0.5, CENTER
    for _ in range(10):
        st, tgt, cmd, wz = step(T); T += dt
    check("5 跟随：θ=+0.5 -> wz>0 同向追赶", wz > 0.1, f"wz={wz:.3f}")

    # ---- 阶段6：遥控器关闭 -> 100ms 内 SAFE_STOP ----
    sticks, rc_on = CENTER, False
    t_lost, t_stop = T, None
    while T - t_lost < 0.5:
        st, tgt, cmd, wz = step(T); T += dt
        if st == gate.STOP and t_stop is None:
            t_stop = T
    check("6 关遥控器 -> SAFE_STOP 且输出清零",
          t_stop is not None and t_stop - t_lost <= RC_TIMEOUT + 2 * dt and all(c == 0 for c in cmd),
          f"反应时间 {(t_stop - t_lost) * 1000:.0f}ms (阈值 {RC_TIMEOUT * 1000:.0f}ms)")

    # ---- 阶段7：急停与人工复位 ----
    rc_on, estop = True, True
    for _ in range(50):
        st, *_ = step(T); T += dt
    check("7a 急停最高优先级 -> SAFE_STOP", st == gate.STOP)
    estop, sticks = False, dict(CENTER, lv=0.5)  # 解除急停但杆没回中
    for _ in range(50):
        st, *_ = step(T); T += dt
    check("7b 杆未回中不自动恢复", st == gate.STOP)
    sticks = CENTER
    for _ in range(50):
        st, *_ = step(T); T += dt
    check("7c 回中+故障清除 -> 重新放行", st == gate.OUT)

    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"\n合计 {len(results)} 项，失败 {n_fail} 项")
    return n_fail == 0

if __name__ == "__main__":
    main()
