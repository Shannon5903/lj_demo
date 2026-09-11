# -*- coding: utf-8 -*-
"""
T2.py —— 根据已知检测点求待定检测点的最优分布范围

模型说明
--------
1) 检测点类型（与 T1 一致）：
   检测范围 = 以检测点为顶点、顶角 2°、方向线为角平分线、两条长边为 1500 的三角形。
   已知检测点 P1 的坐标、方向均确定；待定检测点 P2 的坐标、方向均不确定。

2) 约束：
   a) 待定检测点 P2 与已知检测点检测范围三角形的最短距离 <= 1000；
   b) 两检测范围（按 T1 方法：±1° 直线所夹可行域）形成的多边形直径 <= 40。

3) 枚举：
   - 目标：角很小，简化为目标落在已知检测点方向线上，t∈[0,1500]；
     面概率密度不变 ⇒ 一维密度 ∝ 距离 t（用 t=1500*sqrt((i+0.5)/N) 采样）。
   - 待定点：在约束范围内网格枚举，其方向指向当前目标。

4) 优度评定：
   - 对每个待定点，统计"直径<=40"的目标个数 = 分数；
   - 分数 == 目标总数          ⇒ 最优检测位置；
   - 分数位于前 10%（且不满分）⇒ 良好检测位置。

5) 对称性：只计算方向线一侧 (x>=0)，再关于方向线对称得到另一侧。

结果会保存到 t2_results.json（标准坐标系：已知点在原点、方向为 +y），
供 T2_apply.py 对任意坐标/方向的已知点做刚体变换复用。
"""

import math
import json
import os

import matplotlib.pyplot as plt
import T1

# ============================ 参数 ============================
DETECT_RANGE = 1500.0   # 检测范围三角形长边
MAX_P2_DIST  = 1000.0   # 待定点到已知点检测三角形的最短距离上限
DIAM_LIMIT   = 40.0     # 多边形直径上限
N_TARGETS    = 60       # 目标枚举个数
GRID_STEP    = 20.0     # 待定点网格步长
X_MAX        = 1100.0   # 网格范围（只算方向线右侧，之后对称）
Y_MIN, Y_MAX = 0.0, 1600.0

P1 = (0.0, 0.0, 90.0)   # 已知检测点：(x, y, 方向°)

_BASE = os.path.dirname(os.path.abspath(__file__))
RESULT_FILE = os.path.join(_BASE, "t2_results.json")
FIG_FILE    = os.path.join(_BASE, "t2_result.png")


# ============================ 基础几何 ============================
def _point_segment_distance(px, py, ax, ay, bx, by):
    """点 (px,py) 到线段 (ax,ay)-(bx,by) 的距离"""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return math.hypot(px - ax, py - ay)
    c2 = vx * vx + vy * vy
    if c2 <= c1:
        return math.hypot(px - bx, py - by)
    t = c1 / c2
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def known_triangle_vertices():
    """已知检测点 (0,0,90°) 的检测范围三角形三个顶点"""
    a1 = math.radians(P1[2] - 1.0)
    a2 = math.radians(P1[2] + 1.0)
    return [
        (0.0, 0.0),
        (DETECT_RANGE * math.cos(a1), DETECT_RANGE * math.sin(a1)),
        (DETECT_RANGE * math.cos(a2), DETECT_RANGE * math.sin(a2)),
    ]


def distance_to_known_triangle(px, py, tri):
    """点到已知检测范围三角形的最短距离"""
    (x0, y0), (x1, y1), (x2, y2) = tri
    return min(
        _point_segment_distance(px, py, x0, y0, x1, y1),
        _point_segment_distance(px, py, x1, y1, x2, y2),
        _point_segment_distance(px, py, x2, y2, x0, y0),
    )


def enumerate_targets(n=N_TARGETS):
    """目标在方向线上，密度 ∝ 距离 t：t = 1500*sqrt((i+0.5)/n)"""
    return [DETECT_RANGE * math.sqrt((i + 0.5) / n) for i in range(n)]


# ============================ 评分 ============================
def score_position(x, y, targets):
    """
    待定点 (x,y) 的分数：方向指向各目标，按 T1 方法求两检测范围多边形直径，
    统计直径<=40 的目标个数。
    """
    s = 0
    for t in targets:
        # 待定点方向指向目标 T=(0, t)：向量 (-x, t-y)
        theta2 = math.degrees(math.atan2(t - y, -x)) % 360.0
        verts = T1.compute_feasible_vertices([P1, (x, y, theta2)])
        if len(verts) < 2:
            continue
        hull = T1.convex_hull(verts)
        if len(hull) < 2:
            continue
        d, _ = T1.polygon_diameter(hull)
        if d <= DIAM_LIMIT + 1e-6:
            s += 1
    return s


def compute_scores(targets, tri, verbose=True):
    """在约束范围内网格枚举待定点，返回 [(x, y, score), ...]（只含 x>=0 一侧）"""
    results = []
    y = Y_MIN
    while y <= Y_MAX + 1e-9:
        x = 0.0
        while x <= X_MAX + 1e-9:
            if distance_to_known_triangle(x, y, tri) <= MAX_P2_DIST + 1e-9:
                results.append((x, y, score_position(x, y, targets)))
            x += GRID_STEP
        y += GRID_STEP
        if verbose and abs(y - round(y / GRID_STEP) * GRID_STEP) < 1e-9:
            pass
    return results


def classify(results, n_targets, top_ratio=0.10):
    """把评分结果分成 最优 / 良好（含镜像到方向线另一侧）"""
    # 排序取前 10% 分数线（在 x>=0 一侧排，镜像侧分数相同）
    sorted_scores = sorted((s for _, _, s in results), reverse=True)
    m = len(sorted_scores)
    cutoff_rank = max(1, int(round(top_ratio * m)))
    cutoff = sorted_scores[cutoff_rank - 1]

    optimal = [(x, y) for x, y, s in results if s == n_targets]
    good = [(x, y) for x, y, s in results if cutoff <= s < n_targets]

    # 对称到方向线另一侧
    optimal_mirror = [(x, y) for x, y in optimal] + [(-x, y) for x, y in optimal if x > 0]
    good_mirror = [(x, y) for x, y in good] + [(-x, y) for x, y in good if x > 0]
    return optimal_mirror, good_mirror, cutoff, max(sorted_scores)


# ============================ 绘图与分析 ============================
def analyze(positions, name):
    if not positions:
        print(f"  {name}：无")
        return
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    d = math.hypot(cx, cy)
    # 相对方向线：方向线为 +y 轴，故 x 为垂直距离、y 为沿方向线距离
    print(f"  {name}：{len(positions)} 个")
    print(f"    x 范围 [{min(xs):.1f}, {max(xs):.1f}], y 范围 [{min(ys):.1f}, {max(ys):.1f}]")
    print(f"    质心 = ({cx:.1f}, {cy:.1f})，到已知点距离 = {d:.1f}")
    print(f"    相对方向线：垂直偏移 |x| ≈ {abs(cx):.1f}，沿方向线 ≈ {cy:.1f}")


def plot_positions(optimal, good, tri):
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(7, 9))
    # 已知检测范围三角形 + 方向线
    tx = [p[0] for p in tri] + [tri[0][0]]
    ty = [p[1] for p in tri] + [tri[0][1]]
    ax.plot(tx, ty, color="0.6", lw=1.2, label="已知检测范围")
    ax.plot([0, 0], [0, DETECT_RANGE], color="0.6", ls="--", lw=1, label="方向线")
    ax.scatter([0], [0], marker="*", s=200, c="k", zorder=5, label="已知检测点 (0,0,90°)")

    if good:
        gx = [p[0] for p in good]
        gy = [p[1] for p in good]
        ax.scatter(gx, gy, s=10, c="tab:blue", alpha=0.5, label=f"良好检测位置 ({len(good)})")
    if optimal:
        ox = [p[0] for p in optimal]
        oy = [p[1] for p in optimal]
        ax.scatter(ox, oy, s=25, c="tab:red", label=f"最优检测位置 ({len(optimal)})")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("待定检测点 最优/良好 位置分布")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(FIG_FILE, dpi=120)
    print(f"  图已保存：{FIG_FILE}")
    plt.show()


# ============================ 主流程 ============================
def main():
    print("=" * 60)
    print("T2：待定检测点最优分布范围计算")
    print(f"已知检测点 P1 = {P1}，长边 = {DETECT_RANGE}，直径上限 = {DIAM_LIMIT}")
    print("=" * 60)

    targets = enumerate_targets(N_TARGETS)
    tri = known_triangle_vertices()
    print(f"目标枚举 {N_TARGETS} 个：t ∈ [{targets[0]:.1f}, {targets[-1]:.1f}]（密度∝距离）")

    print("\n网格枚举待定点（只算方向线一侧 x>=0）...")
    results = compute_scores(targets, tri)
    print(f"有效待定点数（一侧）= {len(results)}")

    optimal, good, cutoff, max_score = classify(results, N_TARGETS)
    print(f"\n最高分 = {max_score}/{N_TARGETS}")
    print(f"前10%分数线 = {cutoff}")

    print("\n---- 结果评估 ----")
    analyze(optimal, "最优检测位置")
    analyze(good, "良好检测位置")

    if not optimal:
        # 给出为何"最优(满分)"为空的理论说明
        t_lim = DIAM_LIMIT / (2.0 * math.tan(math.radians(1.0)))
        print("\n说明：最优检测位置（满分）为空。")
        print(f"  两束 2°锥相交，直径下限 ≈ 2·tan1°·|P1T|，故 |P1T| <= {t_lim:.1f} 才可能使直径<=40。")
        print(f"  目标最远到 {DETECT_RANGE}，超出 {t_lim:.1f} 的目标无论待定点如何放置都无法达标，")
        print(f"  因此不存在对全部目标满分的待定点，最高分约为理论上限 {int((t_lim/DETECT_RANGE)**2 * N_TARGETS)}/{N_TARGETS}。")

    # 保存结果（标准坐标系：已知点原点、方向 +y）
    data = {
        "params": {
            "detect_range": DETECT_RANGE,
            "max_p2_dist": MAX_P2_DIST,
            "diam_limit": DIAM_LIMIT,
            "n_targets": N_TARGETS,
            "known_point": list(P1),
        },
        "optimal": [[round(x, 4), round(y, 4)] for x, y in optimal],
        "good": [[round(x, 4), round(y, 4)] for x, y in good],
    }
    with open(RESULT_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"\n结果已保存：{RESULT_FILE}")

    plot_positions(optimal, good, tri)
    print("完成。")


if __name__ == "__main__":
    main()
