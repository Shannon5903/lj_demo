import math
from typing import List, Tuple, Optional

# ============================================================
# 基础数据结构
# ============================================================

class Point2D:
    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)

    def __repr__(self):
        return f"({self.x:.6f}, {self.y:.6f})"

    def distance_to(self, other: 'Point2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


class Line:
    """直线 A*x + B*y + C = 0，法向量 (A,B) 已归一化"""
    def __init__(self, A, B, C):
        n = math.hypot(A, B)
        if n < 1e-15:
            raise ValueError("退化直线")
        self.A, self.B, self.C = A/n, B/n, C/n

    def evaluate(self, p: Point2D) -> float:
        """>0 表示点在法向量所指的一侧"""
        return self.A * p.x + self.B * p.y + self.C

    def intersect(self, other: 'Line', eps=1e-12) -> Optional[Point2D]:
        det = self.A * other.B - other.A * self.B
        if abs(det) < eps:
            return None
        x = (self.B * other.C - other.B * self.C) / det
        y = (self.C * other.A - other.C * self.A) / det
        return Point2D(x, y)



def line_through_point_with_angle(p: Point2D, angle_deg: float) -> Line:
    """
    过点 p、方向角为 angle_deg（相对 +x 轴，逆时针）的直线。
    约定：法向量取 (sin θ, -cos θ) 的相反，使 evaluate > 0 表示"点在直线上方"。
    具体地：直线方向 d = (cosθ, sinθ)，法向量 n 与 d 垂直。
    取 n = (-sinθ, cosθ)，则 evaluate(p0 + t·(0,1)) 的符号 = cosθ。
    当 |cosθ| 较大（线接近水平）时，这个约定稳定。
    为了在任意 θ 下"上/下"都有意义，统一用：
        n = (sinθ, -cosθ)   ->  evaluate > 0 表示"点在直线下方"
        取负号：n = (-sinθ, cosθ) -> evaluate > 0 表示"点在直线上方"
    """
    th = math.radians(angle_deg)
    # 直线方向向量
    dx, dy = math.cos(th), math.sin(th)
    # 法向量（与方向垂直），选择使 +y 侧为 evaluate>0
    # n · (0,1) > 0  =>  B > 0
    A, B = -dy, dx
    if B < 0:           # 保证法向量朝"上"（+y 分量非负）
        A, B = -A, -B
    C = -(A * p.x + B * p.y)
    return Line(A, B, C)



def get_constraint_signs(direction_deg: float) -> Tuple[int, int]:
    """
    返回 (sign_plus, sign_minus)
    sign = +1 : 点必须在 +1°线 之上 (evaluate >= -eps)
    sign = -1 : 点必须在 +1°线 之下 (evaluate <= +eps)
    规则：
      0-89 与 271-360 : +1°线之下(-1), -1°线之上(+1)
      89-91           : 两线之上(+1, +1)
      91-179          : -1°线之下(-1), +1°线之上(+1)
      179-181         : 两线之下(-1, -1)
    """
    d = direction_deg % 360.0
    E = 1e-9  # 边界比较容差（角度）

    if d < 89.0 - E or d > 271.0 + E:
        return (-1, +1)
    if 89.0 - E <= d <= 91.0 + E:
        return (+1, +1)
    if 91.0 + E < d < 179.0 - E:
        return (+1, -1)
    # 179-181
    return (-1, -1)



def compute_feasible_vertices(
    detection_points: List[Tuple[float, float, float]],
    eps: Optional[float] = None,
    dedup_factor: float = 1e-7,
) -> List[Point2D]:
    """
    detection_points: [(x, y, direction_deg), ...]
    返回满足所有约束的候选交点（已去重）。
    """
    if not detection_points:
        return []

    # --- 自适应尺度 ---
    scale = 1.0
    for x, y, _ in detection_points:
        scale = max(scale, abs(x), abs(y))
    if eps is None:
        eps = 1e-9 * scale
    dedup_tol = dedup_factor * scale

    # --- 构建直线与约束 ---
    lines_info = []   # [(plus_line, minus_line, sp, sm)]
    all_lines = []
    for (x, y, d) in detection_points:
        p = Point2D(x, y)
        plus_line  = line_through_point_with_angle(p, d + 1.0)
        minus_line = line_through_point_with_angle(p, d - 1.0)
        sp, sm = get_constraint_signs(d)
        lines_info.append((plus_line, minus_line, sp, sm))
        all_lines.append(plus_line)
        all_lines.append(minus_line)

    # --- 所有交点 ---
    n = len(all_lines)
    candidates = []
    for i in range(n):
        for j in range(i + 1, n):
            pt = all_lines[i].intersect(all_lines[j], eps=1e-12)
            if pt is not None:
                candidates.append(pt)

    # --- 筛选（边界用 ±eps 容差）---
    feasible = []
    for pt in candidates:
        ok = True
        for (plus_line, minus_line, sp, sm) in lines_info:
            vp = plus_line.evaluate(pt)
            vm = minus_line.evaluate(pt)
            # 约束要求 sp*vp >= 0  且 sm*vm >= 0
            if sp * vp < -eps:
                ok = False
                break
            if sm * vm < -eps:
                ok = False
                break
        if ok:
            feasible.append(pt)

    # --- 自适应去重 ---
    unique = []
    for pt in feasible:
        if all(pt.distance_to(q) > dedup_tol for q in unique):
            unique.append(pt)
    return unique


# ============================================================
# 诊断函数：一键定位空集原因
# ============================================================

def diagnose(detection_points, eps=None):
    if not detection_points:
        print("无检测点")
        return
    scale = max([1.0] + [max(abs(x), abs(y)) for x, y, _ in detection_points])
    if eps is None:
        eps = 1e-9 * scale

    lines_info = []
    for (x, y, d) in detection_points:
        p = Point2D(x, y)
        pl = line_through_point_with_angle(p, d + 1.0)
        ml = line_through_point_with_angle(p, d - 1.0)
        sp, sm = get_constraint_signs(d)
        lines_info.append((pl, ml, sp, sm))

    all_lines = []
    for pl, ml, _, _ in lines_info:
        all_lines += [pl, ml]

    print(f"scale={scale:.3e}, eps={eps:.3e}")
    print(f"约束条数 = {2*len(lines_info)}")

    n = len(all_lines)
    n_cand = 0
    n_ok = 0
    sample_fail = None
    for i in range(n):
        for j in range(i + 1, n):
            pt = all_lines[i].intersect(all_lines[j])
            if pt is None:
                continue
            n_cand += 1
            bad = []
            for k, (pl, ml, sp, sm) in enumerate(lines_info):
                vp, vm = pl.evaluate(pt), ml.evaluate(pt)
                if sp * vp < -eps:
                    bad.append((k, "+1线", sp, vp, "+"))
                if sm * vm < -eps:
                    bad.append((k, "-1线", sm, vm, "-"))
            if not bad:
                n_ok += 1
            elif sample_fail is None:
                sample_fail = (pt, bad)

    print(f"候选交点 = {n_cand}, 可行 = {n_ok}")
    if sample_fail:
        pt, bad = sample_fail
        print(f"示例不可行交点 {pt}:")
        for k, tag, s, v, _ in bad:
            need = "≥0" if s > 0 else "≤0"
            print(f"  违反检测点#{k} 的 {tag}: 值={v:+.3e} (要求 {need})")


# ============================================================
# 凸包 & 直径 & 直径圆检验
# ============================================================

def convex_hull(points: List[Point2D]) -> List[Point2D]:
    pts = sorted(points, key=lambda p: (p.x, p.y))
    if len(pts) <= 1:
        return pts[:]

    def cross(o, a, b):
        return (a.x - o.x)*(b.y - o.y) - (a.y - o.y)*(b.x - o.x)

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def polygon_diameter(poly: List[Point2D]):
    n = len(poly)
    if n == 0:
        return 0.0, (None, None)
    if n == 1:
        return 0.0, (poly[0], poly[0])
    best_d, best_pair = -1.0, (poly[0], poly[0])
    for i in range(n):
        for j in range(i + 1, n):
            d = poly[i].distance_to(poly[j])
            if d > best_d:
                best_d, best_pair = d, (poly[i], poly[j])
    return best_d, best_pair


def check_polygon_in_diameter_circle(poly: List[Point2D], eps=1e-6):
    if not poly:
        return True, None, 0.0, []
    d, (p1, p2) = polygon_diameter(poly)
    if p1 is None:
        return True, None, 0.0, []
    center = Point2D((p1.x + p2.x)/2, (p1.y + p2.y)/2)
    r = d/2
    dists = [center.distance_to(p) for p in poly]
    ok = all(v <= r + eps for v in dists)
    return ok, center, r, dists


# ============================================================
# 示例
# ============================================================

def main():
    detection_points = [
        (0.0,  0.0, 45.0),    
        (1000.0, 0.0, 135.0)   
    ]

    print("=" * 60)
    print("检测点：")
    for i, (x, y, d) in enumerate(detection_points):
        print(f"  #{i}: ({x}, {y})  方向 {d}°")

    print("\n" + "=" * 60)
    print("诊断：")
    diagnose(detection_points)

    print("\n" + "=" * 60)
    print("可行顶点：")
    verts = compute_feasible_vertices(detection_points)
    for v in verts:
        print(" ", v)

    if not verts:
        print("可行域为空，无法继续。")
        return

    hull = convex_hull(verts)
    print(f"\n凸包顶点数 = {len(hull)}")
    for v in hull:
        print(" ", v)

    d, (p1, p2) = polygon_diameter(hull)
    print(f"\n直径 = {d:.6f}, 端点 = {p1}, {p2}")

    ok, center, r, dists = check_polygon_in_diameter_circle(hull)
    print(f"圆心 = {center}, 半径 = {r:.6f}")
    for p, v in zip(hull, dists):
        flag = "✓" if v <= r + 1e-6 else "✗"
        print(f"  {flag} {p} 距离圆心 {v:.6f}")
    print("结论：", "全部在圆内 ✓" if ok else "存在顶点在圆外 ✗")


if __name__ == "__main__":
    main()