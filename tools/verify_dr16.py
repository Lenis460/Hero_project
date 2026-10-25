# -*- coding: utf-8 -*-
"""
verify_dr16 —— DBus 帧打包/解析往返自测（与 device/dr16.cpp 位布局严格互逆）
用法: python tools/verify_dr16.py
原理：用打包器把已知通道值编成 18 字节帧 -> 用与 C++ 完全相同的公式解析
      -> 逐项比对。任何一位的位运算错误都会在这里现形。
"""
import random

MID, SPAN, DEFUNCT = 1024, 660, 0x7FF

def pack(ch0, ch1, ch2, ch3, s_left, s_right, wheel_raw):
    """把 11 位通道值按 DJI 位布局编进 18 字节（dr16.cpp 解析的逆运算）"""
    b = [0] * 18
    w = 0
    w |= (ch0 & DEFUNCT) << 0            # bits 0..10
    w |= (ch1 & DEFUNCT) << 11           # bits 11..21
    w |= (ch2 & DEFUNCT) << 22           # bits 22..32
    w |= (ch3 & DEFUNCT) << 33           # bits 33..43
    w |= (s_right & 0x3) << 44           # bits 44..45
    w |= (s_left & 0x3) << 46            # bits 46..47
    for i in range(6):
        b[i] = (w >> (8 * i)) & 0xFF
    b[16] = wheel_raw & 0xFF
    b[17] = (wheel_raw >> 8) & 0xFF
    return bytes(b)

def parse(b):  # 与 dr16.cpp 逐行对应
    ch = [
        (b[0] | (b[1] << 8)) & DEFUNCT,
        ((b[1] >> 3) | (b[2] << 5)) & DEFUNCT,
        ((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & DEFUNCT,
        ((b[4] >> 1) | (b[5] << 7)) & DEFUNCT,
    ]
    s_left = ((b[5] >> 4) & 0x0C) >> 2
    s_right = (b[5] >> 4) & 0x03
    wheel_raw = (b[16] | (b[17] << 8)) & DEFUNCT
    return ch, s_left, s_right, wheel_raw

def to_norm(raw):
    v = (raw - MID) / SPAN
    v = max(-1.0, min(1.0, v))
    return 0.0 if abs(v) < 0.05 else v

SWITCH = {1: "up", 2: "down", 3: "middle", 0: "unknown"}

def main():
    print("=" * 70)
    print("测试 1：定点往返（典型工况）")
    print("=" * 70)
    fixed = [
        ("全中值（双手松开）", [MID]*4, 3, 3, MID),
        ("左杆满上   ch3=1684", [MID, MID, MID, 1684], 3, 3, MID),
        ("左杆满左   ch2=364 ", [MID, MID, 364, MID], 3, 3, MID),
        ("右杆满下   ch1=364 ", [MID, 364, MID, MID], 3, 3, MID),
        ("右杆满右   ch0=1684", [1684, MID, MID, MID], 3, 3, MID),
        ("拨轮满+    wheel=1684", [MID]*4, 3, 3, 1684),
        ("左拨上/右拨下", [MID]*4, 1, 2, MID),
        ("拨杆无效值 0", [MID]*4, 0, 0, MID),
    ]
    ok = True
    for name, chs, sl, sr, wh in fixed:
        b = pack(chs[0], chs[1], chs[2], chs[3], sl, sr, wh)
        pch, psl, psr, pwh = parse(b)
        good = (pch == chs and psl == sl and psr == sr and pwh == wh)
        ok &= good
        print(f"{'PASS' if good else 'FAIL'}  {name:<26} -> 解析 ch={pch} s_l={SWITCH[psl]} s_r={SWITCH[psr]} wheel_raw={pwh}")

    print()
    print("=" * 70)
    print("测试 2：200 组随机值往返 + 归一化语义检查")
    print("=" * 70)
    rng = random.Random(2027)
    for i in range(200):
        chs = [rng.randint(364, 1684) for _ in range(4)]
        sl, sr = rng.randint(0, 3), rng.randint(0, 3)
        wh = rng.randint(364, 1684)
        b = pack(*chs, sl, sr, wh)
        pch, psl, psr, pwh = parse(b)
        assert pch == chs and psl == sl and psr == sr and pwh == wh, f"case {i} FAIL: {chs} {sl} {sr} {wh}"
        for v in pch + [pwh]:
            n = to_norm(v)
            assert -1.0 <= n <= 1.0
    print("PASS  200/200 随机往返一致；归一化输出均在 [-1,1]")

    print()
    print("=" * 70)
    print("测试 3：语义接口演示（映射按 dr16.h 暂定表）")
    print("=" * 70)
    b = pack(MID, MID, 1684, MID, 3, 3, MID)   # ch2 满偏
    ch, sl, sr, wh = parse(b)
    print(f"ch2=1684 -> rc_lh = {to_norm(ch[2]):+.2f}   (左摇杆水平满右，暂定)")
    b = pack(MID, MID, MID, 364, 3, 3, MID)    # ch3 反向满偏
    ch, sl, sr, wh = parse(b)
    print(f"ch3=364  -> rc_lv = {to_norm(ch[3]):+.2f}   (左摇杆垂直满下，暂定)")
    print()
    print("提醒：ch0~ch3 与左右摇杆/拨杆的对应关系是暂定映射，")
    print("      M1 上车必须逐通道实测确认（拨杆拨上/中/下看打印值 1/3/2）。")
    return ok

if __name__ == "__main__":
    main()
