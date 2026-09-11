from T1 import *
import math


# ============================================================
# 参数
# ============================================================
P1 = (0.0, 0.0, 90.0)     # 第一个检测点，检测锥朝上（90°）
DIST_LIMIT = 1500.0        # 第二个检测点到检测锥的最近距离上限
POS_STEP = 10.0           # 第二个检测点的位置网格步长
ANG_STEP = 5.0             # 第二个检测点的方向网格步长（度）
EPS_COVER = 1e-6           # “被直径圆覆盖”判定的数值容差


def frange(lo, hi, step):
    """浮点均匀采样 [lo, hi]（含两端），用整数计数避免累加误差。"""
    n = int(round((hi - lo) / step))
    return [lo + i * step for i in range(n + 1)]


def distance_to_cone(q, p):
    """
    点 q=(x,y) 到检测点 p=(x0,y0,d) 的检测锥（三角锥区域）的最近距离。

    - 若 q 在锥内，返回 0；
    - 否则返回 q 到锥的两条边界射线（方向 d-1°、d+1°）的最小距离：
        投影参数 t<=0 时最近点是锥顶；t>0 时是到该边界直线的垂直距离。
    """
    x0, y0, d = p
    apex = Point2D(x0, y0)
    plus_line = line_through_point_with_angle(apex, d + 1.0)
    minus_line = line_through_point_with_angle(apex, d - 1.0)
    sp, sm = get_constraint_signs(d)

    P = Point2D(q[0], q[1])
    # 在锥内：满足该检测点自身的两条约束
    if sp * plus_line.evaluate(P) >= -1e-9 and sm * minus_line.evaluate(P) >= -1e-9:
        return 0.0

    vx, vy = P.x - x0, P.y - y0
    best = float("inf")
    for da in (d - 1.0, d + 1.0):
        th = math.radians(da)
        ux, uy = math.cos(th), math.sin(th)   # 射线单位方向
        t = vx * ux + vy * uy                 # 沿射线的投影参数
        if t <= 0.0:
            best = min(best, math.hypot(vx, vy))          # 最近点是锥顶
        else:
            best = min(best, abs(vx * uy - vy * ux))      # 到直线的垂直距离
    return best


def prove():
    xs = frange(-DIST_LIMIT, DIST_LIMIT, POS_STEP)
    ys = frange(-DIST_LIMIT, DIST_LIMIT, POS_STEP)
    angs = frange(0.0, 360.0 - ANG_STEP, ANG_STEP)   # 0..355

    n_total = 0        # 实际参与判定的 (位置,方向) 组合数
    n_in_region = 0    # 位置落在“距锥≤1500”区域内的网格点个数
    n_valid = 0        # 构成有效多边形（≥3 顶点）的组合数
    n_degenerate = 0   # 顶点数 1~2（退化为点/线段）
    n_empty = 0        # 可行域为空（两锥不相交）

    max_excess = -1e18
    worst = None

    for x in xs:
        for y in ys:
            if distance_to_cone((x, y), P1) > DIST_LIMIT + 1e-6:
                continue
            n_in_region += 1
            for a in angs:
                n_total += 1
                pts = [P1, (x, y, a)]
                verts = compute_feasible_vertices(pts)
                if not verts:
                    n_empty += 1
                    continue
                hull = convex_hull(verts)
                if len(hull) < 3:
                    n_degenerate += 1
                    continue
                ok, center, r, dists = check_polygon_in_diameter_circle(hull, eps=EPS_COVER)
                excess = max(dists) - r
                n_valid += 1
                if excess > max_excess:
                    max_excess = excess
                    worst = (x, y, a, excess, r, center, hull, dists)

    # ---- 结果输出 ----
    print("=" * 72)
    print("命题：距检测锥 ≤1500 范围内设置第二个检测点，两检测范围构成的")
    print("      多边形一定能被自身直径构成的圆覆盖。")
    print("=" * 72)
    print(f"位置网格：x,y ∈ [{-DIST_LIMIT:.0f}, {DIST_LIMIT:.0f}]，步长 {POS_STEP:g}")
    print(f"方向网格：θ ∈ [0,360)，步长 {ANG_STEP:g}°")
    print(f"落在“距锥≤{DIST_LIMIT:.0f}”内的位置点数：{n_in_region}")
    print(f"参与判定的 (位置,方向) 组合数：{n_total}")
    print(f"  有效多边形（≥3 顶点）：{n_valid}")
    print(f"  退化（1~2 顶点）：{n_degenerate}")
    print(f"  可行域为空：{n_empty}")
    print("-" * 72)

    if n_valid == 0:
        print("没有出现有效多边形，无法检验该命题。")
        return

    print(f"所有有效多边形中，(顶点到圆心最远距离 − 圆半径) 的最大值 = {max_excess:.9f}")
    if max_excess <= EPS_COVER:
        print("结论：在本网格范围内，命题成立 ✓（所有多边形都被自身直径圆覆盖）")
    else:
        print("结论：存在反例 ✗（某个多边形无法被自身直径圆覆盖）")
        x, y, a, excess, r, center, hull, dists = worst
        print(f"  反例：P2=({x},{y},{a})，超出量 {excess:.6f}，圆半径 {r:.6f}")
        print(f"  圆心 {center}")
        for v, dd in zip(hull, dists):
            flag = "✗" if dd > r + EPS_COVER else "✓"
            print(f"    {flag} {v}  距圆心 {dd:.6f}")


if __name__ == "__main__":
    prove()
