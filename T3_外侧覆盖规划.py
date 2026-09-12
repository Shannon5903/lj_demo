# -*- coding: utf-8 -*-
"""
T3.py —— 机器狗清除全向干扰源实践程序（10~16 个干扰源）

程序通过 HTTP 与模拟器链接，导入 T1.py 与 T2_apply.py，实现团队 T3 算法。

核心思想
--------
机器狗可看作一个"探测点"，它在不同位置的探测等价于"不同位置探测点的集合"。
在检测区域（圆心原点、半径 1800 的圆域）内规划若干覆盖圆：

    * 各覆盖圆圆心大致位于"以原点为圆心、半径 1000"的圆上；
    * 覆盖圆半径取 1000（即最小探测距离，保证每处干扰源都落在某个圆内而被探测到）；
    * 覆盖圆的并集覆盖整个检测区域（经理论计算 6~7 个圆即可）。

算法流程
--------
    1) 机器狗沿 y 轴前往第一个规划圆心 (0, 1000)，对全部频道探测；
       清除一干扰源后即不再探测其频道（省时间）。
    2) 依据 T2_apply 做"较优范围"分析：综合求出"路径短、示向度多（较优区域重合多）"
       的位置，移动过去，对满足条件的频道再次探测以锁定目标。
    3) 用 T1 计算多边形直径：
        直径 <= 40 —— 规划路线前往"直径圆圆心"清除；
        直径 >  40 —— 在清除路线上合适位置补测，使直径 < 40 后再清除。
    4) 顺路前往下一个规划圆心，重复上述过程。
    5) 整个清除过程用动态规划（Held-Karp）TSP 优化测量/清除顺序，做到顺路、就近、
       时间最短；在第一个圆心 (0,1000) 处只清一侧，绕一圈回来后再清另一侧。

说明：模拟器通过 measure/clear 请求中的 position 字段隐式移动机器狗（无单独 move
接口），移动代价视为与距离成正比，故 TSP 以几何距离为代价。

顶层参数集中在下方"配置区"，参赛时只需修改队号 ROBOT_ID。
"""

import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from urllib.request import Request, urlopen

import T1
import T2_apply


# ============================ 配置区（参赛时在此修改） ============================
BASE_URL     = "http://127.0.0.1:2026"   # 模拟器地址
ROBOT_ID     = "202609001139"            # 队号（附录给出的队伍参数，参赛时修改）
ARENA_ID     = "default"                 # 竞技场 id

CHANNELS        = list(range(1, 21))     # 频道 1~20（实际干扰源 10~16 个）
DIAM_LIMIT      = 40.0                   # 多边形直径阈值（<=40 可前往圆心精确清除）
DETECT_RANGE    = 1500.0                 # 单次检测作用半径（最大探测距离）
MIN_DETECT_R    = 1000.0                 # 最小探测距离（覆盖圆半径，见核心思想）
OUTER_RADIUS    = 1800.0                 # 检测区域半径（圆心原点）
CENTER_RADIUS   = 1000.0                 # 规划圆圆心距原点的距离

CLEAR_SEARCH_R  = 50.0                   # "near" 时附近搜索清除的半径
DEFAULT_DIST    = 700.0                  # 仅有单条示向度时外推的源距离
MAX_MEASURE     = 6                      # 单个源最多补测次数
OVERLAP_TOL     = 20.0                   # 较优位置"重合"判定容差（与 T2 网格步长一致）
COVER_MARGIN    = 1.0                    # 覆盖圆冗余系数（>=1）
TSP_EXACT_LIMIT = 18                     # 超过此点数退化为最近邻
REQUEST_TIMEOUT = 5.0                    # HTTP 超时（秒）
RETRY_TIMES     = 3                      # 网络故障重试次数
TIME_SAFE_RATIO = 0.95                   # 用掉该比例的剩余时间后提前收尾


# ============================ 模拟器通信 ============================

class SimulatorClient:
    """与模拟器通信的 HTTP 客户端（协议见附件《机器狗请求发送与读取》）。"""

    def __init__(self, base_url=BASE_URL, robot_id=ROBOT_ID, arena_id=ARENA_ID):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.arena_id = arena_id
        self._req = 0

    def _next_id(self, tag):
        self._req += 1
        return f"{tag}-{self._req}-{int(time.time() * 1000)}"

    def _post(self, path, payload):
        url = self.base_url + path
        data = json.dumps(payload).encode("utf-8")
        last_err = None
        for _ in range(RETRY_TIMES):
            request = Request(
                url, data=data,
                headers={"Content-Type": "application/json"}, method="POST",
            )
            try:
                with urlopen(request, timeout=REQUEST_TIMEOUT) as resp:
                    return json.loads(resp.read().decode("utf-8"))
            except Exception as e:  # 网络故障：复用同一 payload（含同一 request_id）重试
                last_err = e
                time.sleep(0.2)
        raise ConnectionError(f"请求 {path} 失败: {last_err}")

    def _base(self, request_id):
        return {"arena_id": self.arena_id, "robot_id": self.robot_id,
                "request_id": request_id}

    def enter(self):
        return self._post("/enter", self._base(self._next_id("enter")))

    def exit(self):
        return self._post("/exit", self._base(self._next_id("exit")))

    def measure(self, x, y, channel):
        payload = self._base(self._next_id("measure"))
        payload["position"] = {"x": x, "y": y}
        payload["channel"] = channel
        return self._post("/measure", payload)

    def clear(self, x, y, channel):
        payload = self._base(self._next_id("clear"))
        payload["position"] = {"x": x, "y": y}
        payload["channel"] = channel
        return self._post("/clear", payload)


def measure_once(client, x, y, ch):
    """返回 ('direction', deg) / ('near', None) / (None, None)。"""
    r = client.measure(x, y, ch)
    kind = r.get("measure_result")
    if kind == "direction":
        try:
            return ("direction", float(r.get("svd_deg")))
        except (TypeError, ValueError):
            return (None, None)
    if kind == "near":
        return ("near", None)
    return (None, None)


def try_near_clear(client, ch, x, y, search_r=CLEAR_SEARCH_R):
    """'near'（距离过近、无示向度）时在附近搜索清除。"""
    if client.clear(x, y, ch).get("clear_result") == "success":
        return True
    for ang in range(0, 360, 45):
        th = math.radians(ang)
        cx, cy = x + search_r * math.cos(th), y + search_r * math.sin(th)
        if client.clear(cx, cy, ch).get("clear_result") == "success":
            return True
    return False


# ============================ 数据结构 ============================

@dataclass
class Source:
    """一个干扰源（对应一个频道）的探测与估计状态。"""
    channel: int
    measurements: List[Tuple[float, float, float]] = field(default_factory=list)  # (x, y, dir)
    est: Optional[Tuple[float, float]] = None      # 估计位置
    diameter: Optional[float] = None               # 当前多边形直径
    cleared: bool = False


# ============================ 几何工具（基于 T1 / T2_apply） ============================

def triangulate(measurements):
    """用 T1 计算可行域凸包直径与直径圆圆心。返回 (diameter, center, radius)。"""
    if len(measurements) < 2:
        return None, None, None
    verts = T1.compute_feasible_vertices(list(measurements))
    if not verts:
        return None, None, None
    hull = T1.convex_hull(verts)
    if len(hull) < 2:
        return None, None, None
    d, _ = T1.polygon_diameter(hull)
    _ok, center, r, _ = T1.check_polygon_in_diameter_circle(hull)
    c = (center.x, center.y) if center is not None else None
    return d, c, r


def intersect_dir_lines(m1, m2):
    """两条 (x,y,dir) 中央方向线的交点（用于位置粗估），平行返回 None。"""
    x1, y1, d1 = m1
    x2, y2, d2 = m2
    u1 = (math.cos(math.radians(d1)), math.sin(math.radians(d1)))
    u2 = (math.cos(math.radians(d2)), math.sin(math.radians(d2)))
    denom = u1[0] * u2[1] - u1[1] * u2[0]
    if abs(denom) < 1e-12:
        return None
    dx, dy = x2 - x1, y2 - y1
    t = (dx * u2[1] - dy * u2[0]) / denom
    return (x1 + t * u1[0], y1 + t * u1[1])


def estimate_position(measurements):
    """由测量集合粗估源位置：方向线交点质心；单条则沿方向外推。"""
    if not measurements:
        return None
    if len(measurements) == 1:
        x, y, d = measurements[0]
        th = math.radians(d)
        return (x + DEFAULT_DIST * math.cos(th), y + DEFAULT_DIST * math.sin(th))
    pts = [p for p in
           (intersect_dir_lines(measurements[i], measurements[j])
            for i in range(len(measurements)) for j in range(i + 1, len(measurements)))
           if p is not None]
    if not pts:
        x, y, d = measurements[-1]
        th = math.radians(d)
        return (x + DEFAULT_DIST * math.cos(th), y + DEFAULT_DIST * math.sin(th))
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def get_good_positions(x0, y0, dir_deg):
    """调用 T2_apply 得到较优二次测量位置（世界坐标）。"""
    try:
        _opt, good = T2_apply.compute_positions(x0, y0, dir_deg)  # opt 为空
    except Exception:
        return []
    return good


def choose_position(candidates, current, prefer=None):
    """从候选位置选一个：优先靠近 prefer（目标估计），其次靠近 current（减少移动）。"""
    best, best_score = None, float("inf")
    for (x, y) in candidates:
        if prefer is not None:
            s = math.hypot(x - prefer[0], y - prefer[1])
        else:
            s = math.hypot(x - current[0], y - current[1])
        if s < best_score:
            best_score, best = s, (x, y)
    return best


def advance_along(x, y, d, dist):
    th = math.radians(d)
    return (x + dist * math.cos(th), y + dist * math.sin(th))


# ============================ TSP（动态规划 / Held-Karp） ============================

def distance_matrix(points):
    n = len(points)
    return [[math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
             for j in range(n)] for i in range(n)]


def nearest_neighbor_order(dist, start_index):
    n = len(dist)
    unvisited = set(range(n))
    unvisited.discard(start_index)
    order, cur, cost = [start_index], start_index, 0.0
    while unvisited:
        nxt = min(unvisited, key=lambda j: dist[cur][j])
        cost += dist[cur][nxt]
        order.append(nxt)
        cur = nxt
        unvisited.discard(nxt)
    return order, cost


def tsp_order(points, start_index=0):
    """
    求从 start_index 出发访问所有点的最短哈密顿路径（Held-Karp 动态规划）。
    返回 (order, cost)，order 为点的下标列表（以 start_index 开头）。
    """
    n = len(points)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [0], 0.0
    dist = distance_matrix(points)
    if n > TSP_EXACT_LIMIT:
        return nearest_neighbor_order(dist, start_index)
    INF = float("inf")
    N = 1 << n
    dp = [[INF] * n for _ in range(N)]
    parent = [[-1] * n for _ in range(N)]
    dp[1 << start_index][start_index] = 0.0
    for mask in range(N):
        for i in range(n):
            cur = dp[mask][i]
            if cur == INF:
                continue
            for j in range(n):
                if (mask >> j) & 1:
                    continue
                nm = mask | (1 << j)
                nd = cur + dist[i][j]
                if nd < dp[nm][j]:
                    dp[nm][j] = nd
                    parent[nm][j] = i
    FULL = N - 1
    best, end = INF, -1
    for i in range(n):
        if dp[FULL][i] < best:
            best, end = dp[FULL][i], i
    order, mask, i = [], FULL, end
    while i != -1:
        order.append(i)
        p = parent[mask][i]
        mask &= ~(1 << i)
        i = p
    order.reverse()
    return order, best


# ============================ 覆盖圆规划 ============================

def plan_circle_centers(R=CENTER_RADIUS, a=MIN_DETECT_R, outer=OUTER_RADIUS,
                        margin=COVER_MARGIN):
    """
    在半径 R 的圆上等角布置覆盖圆圆心，使半径 a 的覆盖圆并集覆盖半径 outer 的圆域。

    覆盖保证（对称情形的极值在 r=0 与 r=outer 处）：
      点 (r,θ) 到最近圆心距离平方 = r² + R² - 2·r·R·cos(π/k)（r 为二次凸函数），
      故只需在边界 r=outer 处满足覆盖即可推出内部全部覆盖，且 r=0 处距离为 R ≤ a。
      角半宽 θ = acos((R²+outer²-a²)/(2·R·outer))，k = ceil(180/θ · margin)。

    返回圆心列表，从 (0, R) 出发沿顺时针方向排列。
    """
    num = (R * R + outer * outer - a * a) / (2.0 * R * outer)
    num = max(-1.0, min(1.0, num))
    theta = math.degrees(math.acos(num))       # 边界处单圆覆盖角半宽
    if theta <= 0.0:
        theta = 1.0
    k = int(math.ceil(180.0 / theta * margin))
    k = max(k, 6)                               # 至少 6 个（团队理论 6~7）
    step = 360.0 / k
    centers = []
    for i in range(k):
        ang = math.radians(90.0 - i * step)     # 从 (0, R) 顺时针
        centers.append((R * math.cos(ang), R * math.sin(ang)))
    return centers


def rel_clockwise_angle(x, y):
    """点相对 (0,1) 方向（正北）的顺时针角，范围 [0,360)。"""
    ang = math.degrees(math.atan2(y, x))
    return (90.0 - ang) % 360.0


# ============================ 较优位置重合分析 ============================

def find_best_overlap_position(good_sets, tolerance=OVERLAP_TOL):
    """
    good_sets: {channel: [(x, y), ...]}，各源（相对其已知检测点）的较优二次测量位置。
    用网格量化求重合频道最多的位置，返回 (最佳位置, 覆盖频道列表)。
    """
    cell = {}
    cell_size = max(tolerance, 1e-6)
    for ch, pts in good_sets.items():
        for (x, y) in pts:
            key = (int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))
            cell.setdefault(key, set()).add(ch)
    if not cell:
        return None, []

    best_key = max(cell, key=lambda k: len(cell[k]))
    bx, by = best_key
    best_channels = set(cell[best_key])
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            key = (bx + dx, by + dy)
            if key in cell:
                best_channels |= cell[key]
    pos = ((bx + 0.5) * cell_size, (by + 0.5) * cell_size)
    return pos, sorted(best_channels)


# ============================ 定位并清除单个源 ============================

def locate_and_clear(client, src, robot_pos):
    """
    对一个源反复测量/三角定位直到直径 <= 40，然后前往直径圆圆心清除。
    返回 (是否成功, 新的机器人位置)。
    """
    ch = src.channel
    meas = list(src.measurements)
    est = src.est if src.est is not None else estimate_position(meas)
    diameter = None

    for _ in range(MAX_MEASURE):
        if len(meas) >= 2:
            diameter, center, _r = triangulate(meas)
            if center is None:
                center = estimate_position(meas)
            est = center
            src.est = est
            src.diameter = diameter
            if diameter is not None and diameter <= DIAM_LIMIT + 1e-6:
                resp = client.clear(center[0], center[1], ch)
                robot_pos = center
                if resp.get("clear_result") == "success":
                    src.cleared = True
                    return True, robot_pos
                # 清除失败：圆心附近无目标，说明估计有偏，就地补测
                kind, deg = measure_once(client, center[0], center[1], ch)
                if kind == "direction":
                    meas.append((center[0], center[1], deg))
                elif kind == "near":
                    if try_near_clear(client, ch, center[0], center[1]):
                        src.cleared = True
                        return True, robot_pos
                    return False, robot_pos
                else:
                    return False, robot_pos
                continue

        # 需要补测：从最后一次测量的较优位置中选下一个测量点
        if not meas:
            return False, robot_pos
        lx, ly, ld = meas[-1]
        candidates = get_good_positions(lx, ly, ld)
        if candidates:
            prefer = est if est is not None else estimate_position(meas)
            nx, ny = choose_position(candidates, (lx, ly), prefer=prefer)
        else:
            nx, ny = advance_along(lx, ly, ld, DEFAULT_DIST * 0.5)

        kind, deg = measure_once(client, nx, ny, ch)
        robot_pos = (nx, ny)
        if kind == "direction":
            meas.append((nx, ny, deg))
        elif kind == "near":
            if try_near_clear(client, ch, nx, ny):
                src.cleared = True
                return True, robot_pos
            mx, my = advance_along(nx, ny, ld, 120.0)
            k2, d2 = measure_once(client, mx, my, ch)
            robot_pos = (mx, my)
            if k2 == "direction":
                meas.append((mx, my, d2))
            else:
                return False, robot_pos
        else:
            # 无信号：可能源已不在范围，改用当前估计尝试清除
            if est is not None:
                resp = client.clear(est[0], est[1], ch)
                robot_pos = est
                if resp.get("clear_result") == "success":
                    src.cleared = True
                    return True, robot_pos
            return False, robot_pos

    # 达到最大补测次数，用当前最佳估计强行清除
    if est is not None:
        resp = client.clear(est[0], est[1], ch)
        robot_pos = est
        if resp.get("clear_result") == "success":
            src.cleared = True
            return True, robot_pos
    return False, robot_pos


# ============================ 探测 / 清除辅助 ============================

def sweep_and_discover(client, sources, cleared, x, y):
    """在位置 (x,y) 探测所有未处理频道，返回新发现源的频道列表。"""
    newly = []
    for ch in CHANNELS:
        if ch in cleared or ch in sources:
            continue
        kind, deg = measure_once(client, x, y, ch)
        if kind == "direction":
            sources[ch] = Source(channel=ch, measurements=[(x, y, deg)])
            newly.append(ch)
        elif kind == "near":
            if try_near_clear(client, ch, x, y):
                cleared.add(ch)
            else:
                # 偏移后重测，试图获取示向度
                for dx, dy in ((100, 0), (-100, 0), (0, 100), (0, -100), (200, 200)):
                    nx, ny = x + dx, y + dy
                    k2, d2 = measure_once(client, nx, ny, ch)
                    if k2 == "direction":
                        sources[ch] = Source(channel=ch, measurements=[(nx, ny, d2)])
                        newly.append(ch)
                        break
                    if k2 == "near":
                        if try_near_clear(client, ch, nx, ny):
                            cleared.add(ch)
                            break
    return newly


def clear_sources(client, sources, cleared, channel_list, robot_pos):
    """按 TSP 顺序清除指定的源集合，返回新的机器人位置。"""
    chs = [ch for ch in channel_list if ch in sources and ch not in cleared]
    if not chs:
        return robot_pos
    for ch in chs:
        if sources[ch].est is None:
            sources[ch].est = estimate_position(sources[ch].measurements)
    pts = [robot_pos] + [sources[ch].est for ch in chs]
    order, cost = tsp_order(pts, start_index=0)
    for idx in order[1:]:
        ch = chs[idx - 1]
        if ch not in sources or ch in cleared:
            continue
        src = sources[ch]
        ok, robot_pos = locate_and_clear(client, src, robot_pos)
        if ok:
            cleared.add(ch)
            sources.pop(ch, None)
    return robot_pos


# ============================ 主流程 ============================

def run(client):
    enter = client.enter()
    if enter.get("accepted") is not True:
        print("进入失败")
        return 0
    remaining = float(enter.get("remaining_real_duration_s", float("inf")))
    deadline = time.time() + remaining * TIME_SAFE_RATIO
    print(f"进入成功，剩余现实时间 {remaining:.1f} 秒")

    centers = plan_circle_centers()
    step = 360.0 / len(centers)
    print(f"规划 {len(centers)} 个覆盖圆圆心（距原点 {CENTER_RADIUS:.0f}）：")
    print("  " + ", ".join(f"({x:.0f},{y:.0f})" for x, y in centers))

    sources = {}          # channel -> Source（已发现、未清除）
    cleared = set()       # 已清除频道
    assigned = {}         # channel -> 应清除它的规划圆心下标

    # ---------- 1) 第一个圆心 (0,1000) 全频道探测 ----------
    robot_pos = centers[0]
    print(f"\n=== 圆心 0 {centers[0]}：全频道探测 ===")
    newly = sweep_and_discover(client, sources, cleared, *robot_pos)
    for ch in newly:
        print(f"  频道 {ch}: 示向度 {sources[ch].measurements[0][2]:.2f}°")

    # ---------- 2) T2_apply 较优位置重合分析，二次锁定 ----------
    if sources:
        good_sets = {}
        for ch, src in sources.items():
            x0, y0, d0 = src.measurements[0]
            good = get_good_positions(x0, y0, d0)
            if good:
                good_sets[ch] = good
        if good_sets:
            pos, covered = find_best_overlap_position(good_sets)
            if pos is not None and len(covered) >= 2:
                print(f"\n=== 较优位置重合分析：移动到 {pos}，覆盖频道 {covered} ===")
                px, py = pos
                robot_pos = (px, py)
                for ch in covered:
                    if ch not in sources:
                        continue
                    kind, deg = measure_once(client, px, py, ch)
                    if kind == "direction":
                        sources[ch].measurements.append((px, py, deg))
                        print(f"  频道 {ch}: 二次示向度 {deg:.2f}°")
                    elif kind == "near":
                        if try_near_clear(client, ch, px, py):
                            cleared.add(ch)
                            sources.pop(ch, None)
                            print(f"  频道 {ch}: 二次探测过近，直接清除")
                    else:
                        print(f"  频道 {ch}: 二次探测无信号")

    # ---------- 3) 依角位置把源分配给各规划圆心（顺路 / 绕一圈） ----------
    for ch, src in sources.items():
        if src.est is None:
            src.est = estimate_position(src.measurements)
        idx = int(round(rel_clockwise_angle(*src.est) / step)) % len(centers)
        assigned[ch] = idx

    # 先清除"分配给当前圆心（即顺时针前向一侧）"的源，另一侧绕回后再清
    print("\n=== 按规划圆顺序顺路清除 ===")
    for i, center in enumerate(centers):
        if i > 0:
            if time.time() > deadline:
                print("剩余时间不足，提前收尾。")
                break
            print(f"\n=== 圆心 {i} {center}：探测未处理频道 ===")
            newly = sweep_and_discover(client, sources, cleared, *center)
            for ch in newly:
                src = sources[ch]
                src.est = estimate_position(src.measurements)
                assigned[ch] = int(round(rel_clockwise_angle(*src.est) / step)) % len(centers)
                print(f"  频道 {ch}: 示向度 {src.measurements[0][2]:.2f}°")

        local = [ch for ch in sources if assigned.get(ch) == i and ch not in cleared]
        if local:
            print(f"  清除该圆心覆盖的频道 {local}")
            robot_pos = clear_sources(client, sources, cleared, local, center)

    # ---------- 4) 兜底：绕完一圈后仍未清除的频道再尝试 ----------
    leftovers = [ch for ch in sources if ch not in cleared]
    if leftovers:
        print(f"\n=== 兜底：清除剩余频道 {leftovers} ===")
        for ch in leftovers:
            src = sources[ch]
            if src.est is None:
                src.est = estimate_position(src.measurements)
        robot_pos = clear_sources(client, sources, cleared, leftovers, robot_pos)

    # ---------- 收尾 ----------
    uncleared = [ch for ch in CHANNELS if ch not in cleared]
    print("\n=== 完成 ===")
    print(f"共清除 {len(cleared)} 个干扰源：{sorted(cleared)}")
    if uncleared:
        print(f"未清除频道：{uncleared}")
    return len(cleared)


def main():
    client = SimulatorClient()
    try:
        run(client)
    finally:
        try:
            resp = client.exit()
            if resp.get("accepted") is True:
                print("退出原因：", resp.get("exit_reason"))
        except Exception as e:
            print("退出请求失败：", e)


if __name__ == "__main__":
    main()
