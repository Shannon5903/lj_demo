# -*- coding: utf-8 -*-
"""
T2_apply.py —— 把 T2 求出的标准结果应用到任意坐标/方向的已知检测点

原理：
    T2.py 在"已知点位于原点、方向为 +y(90°)"的标准坐标系下求出了待定检测点的
    最优/良好位置（已含方向线两侧）。对任意已知点 (x0, y0, dir°)，
    只需做一次旋转 + 平移（刚体变换）即可得到对应的世界坐标位置。

    旋转角 phi = dir - 90°，使标准系的 +y(方向) 转到 dir：
        x' = x0 + x*cos(phi) - y*sin(phi)
        y' = y0 + x*sin(phi) + y*cos(phi)

使用：
    先运行 T2.py 生成 t2_results.json，再调用：
        opt, good = compute_positions(x0, y0, dir_deg)
"""

import json
import math
import os

_BASE = os.path.dirname(os.path.abspath(__file__))
RESULT_FILE = os.path.join(_BASE, "t2_results.json")


def _load_positions():
    """读取 T2 生成的标准坐标系结果，返回 (optimal, good)"""
    if not os.path.exists(RESULT_FILE):
        raise FileNotFoundError(
            f"未找到 {RESULT_FILE}，请先运行 T2.py 生成结果文件。"
        )
    with open(RESULT_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data["optimal"], data["good"]


def _transform(points, x0, y0, dir_deg):
    """把标准坐标系下的点集刚体变换到世界坐标系"""
    phi = math.radians(dir_deg - 90.0)
    c, s = math.cos(phi), math.sin(phi)
    return [(x0 + x * c - y * s, y0 + x * s + y * c) for (x, y) in points]


def compute_positions(x0, y0, dir_deg):
    """
    对任意已知检测点 (x0, y0, dir_deg°)，返回其对应的
    待定检测点 最优位置 与 良好位置（世界坐标，不含方向）。
    返回：(optimal_positions, good_positions)，均为 [(x, y), ...]
    """
    optimal, good = _load_positions()
    return (
        _transform(optimal, x0, y0, dir_deg),
        _transform(good, x0, y0, dir_deg),
    )


def main():
    # 示例：已知检测点位于 (100, 200)，方向 45°
    x0, y0, dir_deg = 100.0, 200.0, 45.0
    opt, good = compute_positions(x0, y0, dir_deg)
    print(f"已知检测点 = ({x0}, {y0}, {dir_deg}°)")
    print(f"待定检测点 最优位置 数量 = {len(opt)}")
    if opt:
        print("  最优位置示例：", opt[:5], "..." if len(opt) > 5 else "")
    print(f"待定检测点 良好位置 数量 = {len(good)}")
    if good:
        print("  良好位置示例：", good[:5], "..." if len(good) > 5 else "")


if __name__ == "__main__":
    main()
