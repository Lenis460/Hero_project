# -*- coding: utf-8 -*-
"""
verify_kinematics —— 麦轮解算/坐标旋转 数值基准（与 algorithm/kinematics.{h,cpp} 同公式）
用法: python tools/verify_kinematics.py
输出各标准测试工况的期望值，用于：
  1) 手算对照学习  2) C++ 实现上板/上机后核对  3) M3 实车方向验证脚本参考
"""
import math

R   = 0.07786   # 轮半径 [m]
A   = 0.275     # a: 中心->前后轮轴线 [m]
B   = 0.225     # b: 中心->左右轮线 [m]
K   = A + B     # 0.5

def rotate_g_to_c(vx_g, vy_g, wz, theta):
    c, s = math.cos(theta), math.sin(theta)
    return (c*vx_g - s*vy_g, s*vx_g + c*vy_g, wz)

def mecanum(vx, vy, wz):
    fl = (vx - vy - K*wz) / R
    fr = (vx + vy + K*wz) / R
    rl = (vx + vy - K*wz) / R
    rr = (vx - vy + K*wz) / R
    return fl, fr, rl, rr

def deg(d): return math.radians(d)

print("=" * 72)
print("测试 1：坐标旋转 v_C = R(θ)·v_G （云台系 -> 底盘系）")
print("=" * 72)
cases = [(0, "云台朝前"), (90, "云台左转90°"), (-90, "云台右转90°"), (180, "云台后转180°")]
for d, name in cases:
    vx, vy, wz = rotate_g_to_c(1.0, 0.0, 0.0, deg(d))
    print(f"θ={d:+4d}° ({name})  推杆[前] -> 底盘系 vx={vx:+.3f} vy={vy:+.3f}  "
          f"({ '车前进' if vx>0.5 else '车后退' if vx<-0.5 else '车左移' if vy>0.5 else '车右移'})")
print()
for d, name in cases:
    vx, vy, wz = rotate_g_to_c(0.0, 1.0, 0.0, deg(d))
    print(f"θ={d:+4d}° ({name})  推杆[左] vy_G=+1 -> 底盘系 vx={vx:+.3f} vy={vy:+.3f}")
print()
for d, name in cases:
    vx, vy, wz = rotate_g_to_c(0.0, -1.0, 0.0, deg(d))
    print(f"θ={d:+4d}° ({name})  推杆[右] vy_G=-1 -> 底盘系 vx={vx:+.3f} vy={vy:+.3f}")

print()
print("=" * 72)
print("测试 2：麦轮逆解算（底盘系速度 -> FL,FR,RL,RR 轮端 rad/s）")
print("=" * 72)
tests = [
    ("(a) 纯前进  vx=1,vy=0,wz=0      ", 1.0, 0.0, 0.0),
    ("(b) 纯左移  vx=0,vy=1,wz=0      ", 0.0, 1.0, 0.0),
    ("(c) 纯逆时针自转 wz=1           ", 0.0, 0.0, 1.0),
    ("(d) 前进+自转 vx=1,vy=0,wz=0.2  ", 1.0, 0.0, 0.2),
]
for name, vx, vy, wz in tests:
    fl, fr, rl, rr = mecanum(vx, vy, wz)
    print(f"{name} FL={fl:+8.3f}  FR={fr:+8.3f}  RL={rl:+8.3f}  RR={rr:+8.3f}")

print()
print("=" * 72)
print("测试 3：减速比换算（M3508 19.2:1，2026-10-25 确认）")
print("=" * 72)
GEAR = 19.2
w_wheel = 1.0
print(f"轮端 1 rad/s -> 电机轴 {w_wheel*GEAR:.1f} rad/s = {w_wheel*GEAR*60/(2*math.pi):.1f} rpm")
v = 1.0
w_w = v / R
print(f"底盘 1 m/s -> 轮端 {w_w:.3f} rad/s -> 电机轴 {w_w*GEAR:.1f} rad/s = "
      f"{w_w*GEAR*60/(2*math.pi):.0f} rpm（设计上限参考）")
rpm = 1000.0
print(f"电机 1000 rpm -> 电机轴 {rpm*2*math.pi/60:.1f} rad/s -> 轮端 "
      f"{rpm*2*math.pi/60/GEAR:.3f} rad/s -> 线速度 {rpm*2*math.pi/60/GEAR*R:.3f} m/s")
print()
print("符号判读提示：")
print("  (b) 纯左移 vy=+1 -> FL,RL 为负（左侧轮向后），FR,RR 为正（右侧轮向前）")
print("  (c) 纯逆时针 wz=+1 -> 左侧轮向后(-)，右侧轮向前(+)，即右侧推着车逆时针转")
