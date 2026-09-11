# -*- coding: utf-8 -*-
"""
T3.py —— 机器狗清除全向干扰源实践程序（10~16 个干扰源）

程序通过 HTTP 与模拟器链接，导入 T1.py 与 T2_apply.py，实现团队 T3 算法：

  阶段一（中心区域，半径 ≈ 1000）：
    1) 机器狗在原点对 1~20 频道探测，得到中心区域内所有干扰源的示向度；
    2) 用 T2_apply 分别求出各示向度对应的"待定检测点较优位置"，
       找出较优区域重合最多的位置（即能同时锁定最多源的位置），移动过去；
    3) 对该位置所覆盖的频道再次探测，用 T1 计算多边形直径：
       - 直径 <= 40：直接前往直径圆圆心清除；
       - 直径  > 40：沿清除路线补测使直径 < 40 后清除；
    4) 按"先近后远、螺旋向外"的顺序（路程就近、路径最短）清理中心区域，
       清理完毕后机器狗位于半径 1000 圆的边界区域。

  阶段二（外围区域，1000 < r <= 1800）：
    5) 在边界区域布置若干覆盖圆（检测半径 1500），使并集覆盖半径 1800 圆域；
    6) 机器狗到达锚点即对剩余频道探测，发现源后在顺路/就近位置用
       T2_apply 求较优位置二次锁定，再到圆心清除。

  全程用动态规划（Held-Karp）TSP 优化测量/清除顺序，做到顺路、就近、时间最短。

说明：模拟器通过 measure/clear 请求中的 position 字段隐式移动机器狗（无单独
move 接口），移动代价视为与距离成正比，故 TSP 以几何距离为代价。

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
BASE_URL   = "http://127.0.0.1:2026"   # 模拟器地址
ROBOT_ID   = "202609001139"            # 队号（附录给出，参赛时改为本队队号）
ARENA_ID   = "default"                 # 竞技场 id

CHANNELS        = list(range(1, 21))   # 频道 1~20
DIAM_LIMIT      = 40.0                 # 多边形直径阈值（<=40 可精确清除）
DETECT_RANGE    = 1500.0               # 单次检测作用半径
CENTER_RADIUS   = 1000.0               # 中心区域半径（螺旋清理到此边界）
OUTER_RADIUS    = 1800.0               # 整体目标区域半径
NEAR_CLEAR_R    = 50.0                 # "near" 时清除搜索半径
DEFAULT_DIST    = 700.0                # 仅有单条示向度时外推的源距离
MAX_MEASURE     = 6                    # 单个源最多补测次数
OVERLAP_TOL     = 20.0                 # 较优位置"重合"判定容差（与 T2 网格步长一致）
COVER_MARGIN    = 1.5                  # 外围覆盖圆冗余系数
TSP_EXACT_LIMIT = 18                   # 超过此点数退化为最近邻
REQUEST_TIMEOUT = 5.0                  # HTTP 超时（秒）
RETRY_TIMES     = 3                    # 网络故障重试次数
TIME_SAFE_RATIO = 0.95                 # 用掉该比例的剩余时间后提前收尾


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


# ============================ 数据结构 ============================

@dataclass
class Source:
    """一个干扰源（对应一个频道）的探测与估计状态。"""
    channel: int
    measurements: List[Tuple[float, float, float]] = field(default_factory=list)  # (x, y, dir)
    est: Optional[Tuple[float, float]] = None      # 估计位置
    diameter: Optional[float] = None               # 当前多边形直径
    cleared: bool = False


# ============================ 几何工具（基于 T1） ============================

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
    ok, center, r, _ = T1.check_polygon_in_diameter_circle(hull)
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
        opt, good = T2_apply.compute_positions(x0, y0, dir_deg)
    except Exception:
        return []
    return opt if opt else good


def choose_position(candidates, current, prefer=None):
    """从候选位置选一个：优先靠近 prefer(目标估计)，其次靠近 current(减少移动)。"""
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


def try_near_clear(client, ch, x, y, search_r=NEAR_CLEAR_R):
    """'near'（距离过近、无示向度）时在附近搜索清除。"""
    if client.clear(x, y, ch).get("clear_result") == "success":
        return True
    for ang in range(0, 360, 45):
        th = math.radians(ang)
        cx, cy = x + search_r * math.cos(th), y + search_r * math.sin(th)
        if client.clear(cx, cy, ch).get("clear_result") == "success":
            return True
    return False


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


# ============================ 覆盖圆规划（阶段二） ============================

def plan_anchor_positions(outer=OUTER_RADIUS, detect=DETECT_RANGE,
                          center=CENTER_RADIUS, margin=COVER_MARGIN):
    """
    在半径 center 的边界圆上等角布置锚点，使锚点检测半径(1500)的并集
    覆盖半径 outer 的圆域。角间距由最远处(r=outer)的两锚点覆盖弧长确定。
    """
    R, r = center, outer
    num = (R * R + r * r - detect * detect) / (2.0 * R * r)
    num = max(-1.0, min(1.0, num))
    theta = math.degrees(math.acos(num))
    if theta <= 0.0:
        theta = 1.0
    k = int(math.ceil(360.0 / (2.0 * theta) * margin))
    k = max(k, 6)
    anchors = []
    for i in range(k):
        ang = math.radians(360.0 * i / k)
        anchors.append((R * math.cos(ang), R * math.sin(ang)))
    return anchors


# ============================ 定位并清除单个源 ============================

def locate_and_clear(client, src, robot_pos):
    """
    对一个源反复测量/三角定位直到直径 <= 40，然后前往圆心清除。
    返回 (是否成功, 新的机器人位置)。
    """
    ch = src.channel
    meas = list(src.measurements)
    est = src.est
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
            mx, my = advance_along(nx, ny, ld, 100.0)
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


# ============================ 较优位置重合分析 ============================

def find_best_overlap_position(good_sets, tolerance=OVERLAP_TOL):
    """
    good_sets: {channel: [(x, y), ...]}，各源（相对已知点）的较优二次测量位置。
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


# ============================ 主流程 ============================

def run(client):
    enter = client.enter()
    if enter.get("accepted") is not True:
        print("进入失败")
        return
    remaining = float(enter.get("remaining_real_duration_s", float("inf")))
    deadline = time.time() + remaining * TIME_SAFE_RATIO
    print(f"进入成功，剩余现实时间 {remaining:.1f} 秒")

    robot_pos = (0.0, 0.0)
    sources = {}                 # channel -> Source
    cleared_channels = set()

    # ---------- 阶段一：原点全频道探测 ----------
    print("\n=== 阶段一：原点 1~20 频道探测 ===")
    for ch in CHANNELS:
        kind, deg = measure_once(client, 0.0, 0.0, ch)
        if kind == "direction":
            sources[ch] = Source(channel=ch, measurements=[(0.0, 0.0, deg)])
            print(f"  频道 {ch}: 示向度 {deg:.2f}°")
        elif kind == "near":
            print(f"  频道 {ch}: 距离过近，尝试直接清除")
            if try_near_clear(client, ch, 0.0, 0.0):
                cleared_channels.add(ch)
                print(f"    频道 {ch} 已清除")
            else:
                # 近但未清：偏移后重新测量获取方向
                mx, my = 50.0, 50.0
                robot_pos = (mx, my)
                k2, d2 = measure_once(client, mx, my, ch)
                if k2 == "direction":
                    sources[ch] = Source(channel=ch, measurements=[(mx, my, d2)])
                    print(f"    频道 {ch} 偏移后示向度 {d2:.2f}°")
                elif k2 == "near":
                    if try_near_clear(client, ch, mx, my):
                        cleared_channels.add(ch)
        else:
            print(f"  频道 {ch}: 无信号")

    if not sources:
        print("中心区域未发现任何干扰源。")
    else:
        # 用 T2_apply 求各源的较优二次测量位置，找重合最多的位置
        good_sets = {}
        for ch, src in sources.items():
            x0, y0, d0 = src.measurements[0]
            good = get_good_positions(x0, y0, d0)
            if good:
                good_sets[ch] = good

        if good_sets:
            pos, covered = find_best_overlap_position(good_sets)
            print(f"\n选择重合最多的二次测量位置 {pos}，覆盖频道 {covered}")
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
                        cleared_channels.add(ch)
                        sources[ch].cleared = True
                        print(f"  频道 {ch}: 二次探测过近，直接清除")
                else:
                    print(f"  频道 {ch}: 二次探测无信号")

        # 先近后远排序 + TSP 规划清除顺序
        pending = [s for s in sources.values()
                   if not s.cleared and s.channel not in cleared_channels]
        for s in pending:
            if s.est is None:
                s.est = estimate_position(s.measurements)
        pending.sort(key=lambda s: (s.est[0] ** 2 + s.est[1] ** 2) if s.est else float("inf"))

        if pending:
            all_pts = [robot_pos] + [s.est for s in pending]
            order, cost = tsp_order(all_pts, start_index=0)
            visit_seq = [pending[i - 1] for i in order[1:]]
            print(f"\n=== 阶段一：清除中心区域（TSP 顺序，路径长度 {cost:.1f}）===")
            failed = []
            for s in visit_seq:
                ok, robot_pos = locate_and_clear(client, s, robot_pos)
                if ok:
                    cleared_channels.add(s.channel)
                    print(f"  频道 {s.channel} 已清除（估计 {s.est}）")
                else:
                    failed.append(s)
            for s in failed:  # 失败重试一次
                ok, robot_pos = locate_and_clear(client, s, robot_pos)
                if ok:
                    cleared_channels.add(s.channel)
                    print(f"  频道 {s.channel} 重试后已清除")
                else:
                    print(f"  频道 {s.channel} 最终未清除")

    # ---------- 阶段二：外围区域覆盖 ----------
    remaining_channels = [ch for ch in CHANNELS if ch not in cleared_channels]
    anchors = plan_anchor_positions()
    print(f"\n=== 阶段二：外围区域覆盖（{len(anchors)} 个锚点）===")

    if remaining_channels and anchors:
        anchor_pts = [robot_pos] + anchors
        order, cost = tsp_order(anchor_pts, start_index=0)
        visit = [anchors[i - 1] for i in order[1:]]

        for (ax, ay) in visit:
            if time.time() > deadline:
                print("剩余时间不足，提前收尾。")
                break
            robot_pos = (ax, ay)
            newly = {}
            for ch in remaining_channels:
                kind, deg = measure_once(client, ax, ay, ch)
                if kind == "direction":
                    newly[ch] = (ax, ay, deg)
                    print(f"  锚点({ax:.0f},{ay:.0f}) 频道 {ch}: 示向度 {deg:.2f}°")
                elif kind == "near":
                    if try_near_clear(client, ch, ax, ay):
                        cleared_channels.add(ch)
                        print(f"  锚点 频道 {ch}: 过近，直接清除")
            # 对发现的源二次锁定并清除（顺路/就近）
            for ch, m in newly.items():
                if ch in cleared_channels:
                    continue
                src = Source(channel=ch, measurements=[m])
                src.est = estimate_position(src.measurements)
                ok, robot_pos = locate_and_clear(client, src, robot_pos)
                if ok:
                    cleared_channels.add(ch)
                    remaining_channels.remove(ch)
                    print(f"  频道 {ch} 已清除")
                elif time.time() > deadline:
                    break

    # ---------- 收尾 ----------
    uncleared = [ch for ch in CHANNELS if ch not in cleared_channels]
    print("\n=== 完成 ===")
    print(f"共清除 {len(cleared_channels)} 个干扰源：{sorted(cleared_channels)}")
    if uncleared:
        print(f"未清除频道：{uncleared}")
    return len(cleared_channels)


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
