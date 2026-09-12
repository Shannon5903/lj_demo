# -*- coding: utf-8 -*-
"""
T3_tx.py —— 问题3：全向干扰源（10~16 个，频道 1~20，个数未知）的自动搜索、定位与清除

================================================================================
一、总体策略（团队 T3 算法）
================================================================================
把机器狗在每个停留点看作一个“检测点”。

1) 覆盖规划（保证“一定能发现全部干扰源”）
   检测区域是原点为圆心、半径 ARENA_R=1800 m 的圆；每个全向干扰源的有效接收半径
   ≥1000 m。在以原点为圆心、半径 PLAN_R=999 m 的圆上均布 N_STATIONS=7 个“扫描站位”
   （第一个站位在 +y 轴上，即 (0,999)≈(0,1000)）。可数值验证：区域内任意一点到最近
   站位的距离 ≤ 999 m < 1000 m，因此 7 个以站位为圆心、1000 m 为半径的“探测圆”的并集
   100% 覆盖整个检测区域（注意：没有任何一个规划圆的圆心在原点）。
   => 只要依次在 7 个站位对“尚未清除的频道”各检测一次，任意干扰源至少会被发现一次。

2) 二次定位（T2_apply）与交会区域（T1）
   在站位首次测到某频道示向度后，用 T2_apply.compute_positions 得到该干扰源“第二检测
   点”的良好候选区域，结合全局路径就近挑选一个第二检测点；两次及以上检测后，用与角度
   无关的“左/右半平面叉积”严格构造各 ±1° 楔形（feasible_vertices，规避按象限判上下在
   181°~271°方向上的符号缺口），再复用 T1 的 convex_hull / polygon_diameter /
   check_polygon_in_diameter_circle 计算交会多边形、直径与直径圆，并用最小外接圆（MEC，
   直径圆能包住时直接用直径圆圆心）得到清除瞄准点。
   —— 可行域是各 ±1° 楔形半平面的交集，因而是凸多边形；一旦有界（凸包≥3 顶点），每增加
      一次检测，可行域只会缩小、不会变大，真实干扰源始终位于该多边形内。

3) 锁定与清除
   - 多边形“有界 且 直径≤DIAM_LIMIT(40) 且 外接圆半径≤SAFE_CLEAR_R(<清除半径20)”时
     判定锁定：前往瞄准点 /clear（20 m 内必清除成功，因为真实源一定在 MEC 圆盘内）。
   - 否则在前往站位/清除点的途中，于合适位置补测（第二点用 T2 良好区，三次及以上用相对
     当前外接圆圆心约 550 m 的环形候选点，保证可接收到信号且交会夹角≥35°），直到直径达标。

4) 路径规划：贪心（阈值触发 + 最近邻）为主，2-opt 局部优化；尽量顺路、就近、时间最短。
   扫描站位构成覆盖“主干”，锁定/清除任务在“顺路阈值”内时插队就近处理，否则继续推进站位。

================================================================================
二、与模拟器的连接（依据附件1、附件2）
================================================================================
HTTP+JSON，POST 到 BASE_URL 的 /enter、/measure、/clear、/exit 四条接口；
逐次串行、每个新动作使用新 request_id（网络故障重试同一动作时复用同一 request_id 与请求体）；
同时检查 HTTP 状态与响应 accepted 字段。顶层 ROBOT_ID 必须改为当前登录参赛队号。

用法：
    python T3_tx.py                # 连接本机模拟器正式/演练运行（先在模拟器里开放接口）
    python T3_tx.py --self-test    # 离线自测：内置随机全向干扰源模拟器，验证策略能否全清
    python T3_tx.py --self-test -n 200   # 离线连跑 200 个随机案例并统计成功率/平均虚拟耗时
================================================================================
"""

import os
import sys
import json
import math
import time
import random
import argparse
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

# ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# ||========================  顶层参数（按需修改）  ==========================||
# ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

# —— 附录对应：参赛队号（必须与模拟器当前登录队号逐字节一致）——
ROBOT_ID = "<参赛队号>"
# —— 模拟器本机接口地址（空闲时可在模拟器中改端口，这里同步改）——
BASE_URL = "http://127.0.0.1:2026"
ARENA_ID = "default"

# T1.py / T2_apply.py / t2_results.json 所在目录（只读使用，不修改其中任何文件）
LJ_CODE_DIR = r"D:\26建模国赛\lj_demo-main\lj_demo-main"

# —— 场景几何常量（来自题目与附录2）——
ARENA_R = 1800.0          # 检测区域半径 m
EFF_R_MIN = 1000.0        # 干扰源有效接收半径下界 m
PLAN_R = 999.0            # 扫描站位环半径 m（取999：最坏点距站位999<1000，严格留余量；1000为临界）
N_STATIONS = 7            # 扫描站位个数（7 个探测圆即可 100% 覆盖）
FIRST_STATION_ANGLE_DEG = 90.0   # 第一个站位在 +y 轴（沿 y 轴前往 (0,~1000)）

# —— 动作耗时（秒，虚拟时间）——
SPEED = 5.0               # 机器狗移动速度 m/s
MEASURE_T = 5.0           # 一次检测耗时
SWITCH_T = 1.0            # 频道切换耗时
CLEAR_T_OK = 5.0          # 清除成功耗时（光学定位3 + 激光2）
CLEAR_T_MISS = 3.0        # 清除未发现耗时（仅光学定位3）
NEAR_R = 5.0              # 距离过近阈值
CLEAR_R = 20.0            # 清除半径

# —— 定位/锁定阈值 ——
DIAM_LIMIT = 40.0         # T1/T2：定位多边形直径目标上限
SAFE_CLEAR_R = 18.5       # 锁定所需的外接圆半径上限（<清除半径20，留安全余量）
MIN_BASELINE = 200.0      # 两次检测点之间的最小基线距离 m
RING_R = 550.0            # 三次及以上补测点相对当前区域圆心的首选环形半径 m（越近定位越准）
RING_GUARD = 980.0        # 保证补测点能收到信号：到区域圆心距离 + 区域半径 ≤ 该值(<1000)
GOOD_ANGLE_DEG = 35.0     # 补测视线与已有视线的理想最小交会角
GOOD_SIN = math.sin(math.radians(GOOD_ANGLE_DEG))
MAX_COORD = 2100.0        # 规划用坐标不超过该值（接口硬上限 2,000,000，这里远更严格）

# —— 贪心路径规划阈值 ——
DETOUR_TOL = 300.0        # “顺路”阈值：相对直奔站位的绕行量不超过该值则插队处理 m
NEAR_TOL = 300.0          # “就近”阈值：离当前点小于该值也优先处理 m

# —— 运行保护 ——
HTTP_TIMEOUT = 5.0        # 单次 HTTP 超时 s
HTTP_RETRY = 6            # 网络故障最大重试次数
RETRY_BACKOFF = 0.8       # 重试退避基数 s
RESERVE_EXIT_S = 20.0     # 预留多少现实秒用于收尾 /exit
MAX_ACTIONS = 3000        # 动作数硬上限，防止异常高频循环（也避免日志超 2MB）
ALL_CHANNELS = list(range(1, 21))


# ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# ||====================  载入 T1、T2_apply（只读复用）  =====================||
# ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
if LJ_CODE_DIR not in sys.path:
    sys.path.insert(0, LJ_CODE_DIR)
import T1                                  # noqa: E402
import T2_apply                            # noqa: E402


# ============================================================================
# 一、通用几何工具
# ============================================================================
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _circumcircle(a, b, c):
    """三点外接圆，返回 (cx, cy, r)；三点共线时返回 None。"""
    ax, ay = a; bx, by = b; cx, cy = c
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return ux, uy, math.hypot(ax - ux, ay - uy)


def _circle_from_two(a, b):
    cx, cy = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
    return cx, cy, dist(a, b) / 2.0


def _in_circle(circ, p, eps=1e-9):
    if circ is None:
        return False
    cx, cy, r = circ
    return math.hypot(p[0] - cx, p[1] - cy) <= r + eps * max(1.0, r)


def minimal_enclosing_circle(points):
    """Welzl 最小外接圆（随机增量实现，支持浮点）。points: [(x,y),...]。"""
    pts = [(float(p[0]), float(p[1])) for p in points]
    random.shuffle(pts)

    def welzl(P, n, R):
        if n == 0 or len(R) == 3:
            return mec_from_boundary(R)
        p = P[n - 1]
        c = welzl(P, n - 1, R)
        if _in_circle(c, p):
            return c
        return welzl(P, n - 1, R + [p])

    def mec_from_boundary(R):
        if not R:
            return (0.0, 0.0, 0.0)
        if len(R) == 1:
            return (R[0][0], R[0][1], 0.0)
        if len(R) == 2:
            return _circle_from_two(R[0], R[1])
        c = _circumcircle(R[0], R[1], R[2])
        if c is not None:
            return c
        # 三点共线退化为直径最长的两点圆
        best = None
        for i in range(3):
            for j in range(i + 1, 3):
                cc = _circle_from_two(R[i], R[j])
                if best is None or cc[2] > best[2]:
                    best = cc
        return best

    if not pts:
        return (0.0, 0.0, 0.0)
    return welzl(pts, len(pts), [])


def path_length(seq):
    return sum(dist(seq[i - 1], seq[i]) for i in range(1, len(seq)))


def two_opt(seq, passes=40):
    """开放路径（首点固定）2-opt 局部优化，返回更短的点序列。"""
    best = list(seq)
    n = len(best)
    improved = True
    it = 0
    while improved and it < passes:
        improved = False
        it += 1
        for i in range(1, n - 1):           # 首点固定，不动 0
            for j in range(i + 1, n):       # 反转 [i,j]
                if j - i < 1:
                    continue
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                if path_length(cand) + 1e-9 < path_length(best):
                    best = cand
                    improved = True
    return best


def nn_then_2opt(start, goals):
    """从 start 出发，对 goals=(key,(x,y)) 列表先最近邻成链，再 2-opt 优化。"""
    remaining = list(goals)
    order = []
    cur = start
    while remaining:
        k = min(range(len(remaining)), key=lambda i: dist(cur, remaining[i][1]))
        item = remaining.pop(k)
        order.append(item)
        cur = item[1]
    pts = [start] + [g[1] for g in order]
    pts = two_opt(pts)
    # 用优化后的坐标顺序回填 key
    keymap = {}
    for key, xy in order:
        keymap[(round(xy[0], 6), round(xy[1], 6))] = key
    opt_keys = [keymap[(round(p[0], 6), round(p[1], 6))] for p in pts[1:]]
    return opt_keys


def _wedge_constraints(det):
    """
    对一次检测 (px,py,svd°) 构造 [svd-1°,svd+1°] 楔形的两条半平面边界（与角度无关，
    用“有向直线左/右侧叉积”严格构造，对 0~360 任意方向都正确，规避按象限判符号的缺口）。
    返回两条归一化约束 (A,B,C,s)，表示 s*(A*x+B*y+C) >= 0。
    """
    px, py, th = det
    out = []
    for ang, side in ((th - 1.0, +1), (th + 1.0, -1)):
        a = math.radians(ang)
        dx, dy = math.cos(a), math.sin(a)
        # cross(d, X-P) = -dy*x + dx*y + dy*px - dx*py ；左(+1)/右(-1)
        A, B, C = -dy, dx, dy * px - dx * py
        n = math.hypot(A, B)
        out.append((A / n, B / n, C / n, side))
    return out


def feasible_vertices(dets, dedup_factor=1e-7):
    """正确版本的交会可行域顶点：所有楔形半平面交集的全部边界交点（浮点容差+去重）。"""
    if len(dets) < 2:
        return []
    scale = max([1.0] + [max(abs(x), abs(y)) for x, y, _ in dets])
    eps = 1e-9 * scale
    tol = dedup_factor * scale
    cons, lines = [], []
    for det in dets:
        for c in _wedge_constraints(det):
            cons.append(c)
            lines.append(c[:3])
    cands = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            A1, B1, C1 = lines[i]
            A2, B2, C2 = lines[j]
            detm = A1 * B2 - A2 * B1
            if abs(detm) < 1e-12:
                continue
            x = (B1 * C2 - B2 * C1) / detm
            y = (C1 * A2 - C2 * A1) / detm
            if all(s * (A * x + B * y + C) >= -eps for A, B, C, s in cons):
                cands.append(T1.Point2D(x, y))
    uniq = []
    for p in cands:
        if all(p.distance_to(q) > tol for q in uniq):
            uniq.append(p)
    return uniq


def build_stations():
    """生成 N_STATIONS 个等距扫描站位，首个位于 +y 轴。"""
    pts = []
    for k in range(N_STATIONS):
        a = math.radians(FIRST_STATION_ANGLE_DEG + k * 360.0 / N_STATIONS)
        pts.append((PLAN_R * math.cos(a), PLAN_R * math.sin(a)))
    return pts


_COV_WORST_CACHE = None


def coverage_worst_distance(sample=700):
    """数值自检：检测区域内任一点到最近扫描站位的最大距离（应 < EFF_R_MIN），结果缓存。"""
    global _COV_WORST_CACHE
    if _COV_WORST_CACHE is not None:
        return _COV_WORST_CACHE
    sts = build_stations()
    worst = 0.0
    for ir in range(sample + 1):
        r = ARENA_R * ir / sample
        for it in range(sample):
            a = 2 * math.pi * it / sample
            x, y = r * math.cos(a), r * math.sin(a)
            dm = min(dist((x, y), s) for s in sts)
            worst = max(worst, dm)
    _COV_WORST_CACHE = worst
    return worst


# ============================================================================
# 二、通信客户端
# ============================================================================
class RealSimClient:
    """依据附件2实现的真实模拟器客户端（串行、幂等、同时校验 HTTP 与 accepted）。"""

    def __init__(self, base_url=BASE_URL, robot_id=ROBOT_ID):
        self.base_url = base_url
        self.robot_id = robot_id

    def _post_once(self, path, payload):
        req = Request(
            self.base_url + path,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        with urlopen(req, timeout=HTTP_TIMEOUT) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def send(self, path, payload, rid):
        """发送一个动作；网络故障时用同一 request_id 与同一请求体重试。"""
        last_err = None
        for attempt in range(HTTP_RETRY):
            try:
                return self._post_once(path, payload)
            except HTTPError as e:  # 能形成 HTTP 响应的错误，尽量解析 JSON
                try:
                    return json.loads(e.read().decode("utf-8"))
                except Exception:
                    last_err = e
            except (URLError, TimeoutError, ConnectionError, OSError) as e:
                last_err = e  # 接口未开放/连接被关闭：退避后用同一 rid 重试
            time.sleep(RETRY_BACKOFF * (attempt + 1))
        raise ConnectionError(f"{path} 连续 {HTTP_RETRY} 次通信失败：{last_err}")

    # —— 四条指令 ——
    def enter(self, rid):
        return self.send("/enter",
                         {"arena_id": ARENA_ID, "robot_id": self.robot_id, "request_id": rid}, rid)

    def exit(self, rid):
        return self.send("/exit",
                         {"arena_id": ARENA_ID, "robot_id": self.robot_id, "request_id": rid}, rid)

    def measure(self, rid, x, y, ch):
        return self.send("/measure", {
            "arena_id": ARENA_ID, "robot_id": self.robot_id, "request_id": rid,
            "position": {"x": float(x), "y": float(y)}, "channel": int(ch)}, rid)

    def clear(self, rid, x, y, ch):
        return self.send("/clear", {
            "arena_id": ARENA_ID, "robot_id": self.robot_id, "request_id": rid,
            "position": {"x": float(x), "y": float(y)}, "channel": int(ch)}, rid)


class MockSimClient:
    """离线自测用：模拟问题3的全向干扰源电磁环境与计时（不联网）。"""

    def __init__(self, n_min=10, n_max=16, seed=None):
        rng = random.Random(seed)
        self.rng = rng
        n = rng.randint(n_min, n_max)
        chans = rng.sample(ALL_CHANNELS, n)
        self.sources = {}
        for ch in chans:  # 均匀分布于半径1800圆内，有效接收半径1000~1500
            r = ARENA_R * math.sqrt(rng.random())
            a = 2 * math.pi * rng.random()
            self.sources[ch] = {
                "xy": (r * math.cos(a), r * math.sin(a)),
                "R": rng.uniform(1000.0, 1500.0),
            }
        self.cleared = set()
        self.cur_pos = (0.0, 0.0)
        self.cur_ch = 1
        self.vt = 0.0

    def _err(self, ch, x, y):  # 同一地点误差固定（随位置变化呈统计规律）
        h = (ch * 1000003 + int(round(x / 3.0)) * 9176 + int(round(y / 3.0)) * 69061) % 100000
        return (h / 100000.0) * 2.0 - 1.0

    def _advance_move(self, x, y):
        self.vt += dist(self.cur_pos, (x, y)) / SPEED
        self.cur_pos = (x, y)

    def measure(self, rid, x, y, ch):
        self._advance_move(x, y)
        if ch != self.cur_ch:
            self.vt += SWITCH_T
            self.cur_ch = ch
        self.vt += MEASURE_T
        out = {"accepted": True, "virtual_time_s": self.vt}
        src = self.sources.get(ch)
        if src is None or ch in self.cleared:
            out["measure_result"] = "no_signal"
            return out
        gx, gy = src["xy"]
        d = dist((x, y), (gx, gy))
        if d > src["R"]:
            out["measure_result"] = "no_signal"
        elif d <= NEAR_R:
            out["measure_result"] = "near"
        else:
            true_ang = math.degrees(math.atan2(gy - y, gx - x)) % 360.0
            svd = (true_ang + self._err(ch, x, y)) % 360.0
            out["measure_result"] = "direction"
            out["svd_deg"] = round(svd, 2)
        return out

    def clear(self, rid, x, y, ch):
        self._advance_move(x, y)
        out = {"accepted": True}
        src = self.sources.get(ch)
        if src is not None and ch not in self.cleared and dist((x, y), src["xy"]) <= CLEAR_R:
            self.cleared.add(ch)
            self.vt += CLEAR_T_OK
            out["clear_result"] = "success"
        else:
            self.vt += CLEAR_T_MISS
            out["clear_result"] = "no_target_in_range"
        out["virtual_time_s"] = self.vt
        return out

    def enter(self, rid):
        return {"accepted": True, "virtual_time_s": 0.0,
                "remaining_real_duration_s": 1200, "max_real_duration_s": 1200}

    def exit(self, rid):
        return {"accepted": True, "virtual_time_s": self.vt, "exit_reason": "user_exit"}


# ============================================================================
# 三、单频道跟踪状态
# ============================================================================
class Track:
    def __init__(self, ch):
        self.ch = ch
        self.dets = []              # [(x,y,svd_deg), ...]
        self.tried = set()          # 已测但 no_signal 的候选点（取整去重）
        self.bounded = False        # 交会区域是否有界（凸包≥3顶点）
        self.diam = None
        self.mec = None             # (cx,cy,r)
        self.target = None          # 清除瞄准点
        self.state = "open"         # open / tight
        self.near = False
        self.clear_attempts = 0
        self.probe = False          # 清除失败后需要原地补测
        self.last_mec = None        # 最近一次“有界区域”的最小外接圆 (cx,cy,r)
        self.last_diam = None       # 最近一次有界区域直径
        self.stall = 0              # 连续未明显收敛的补测次数（用于逃逸）

    @staticmethod
    def _key(q):
        return (round(q[0], 1), round(q[1], 1))


# ============================================================================
# 四、任务主控
# ============================================================================
class Mission:
    def __init__(self, client, verbose=True, log_path=None):
        self.c = client
        self.verbose = verbose
        self.stations = build_stations()
        self.station_idx = 0
        self.tracks = {}
        self.cleared = set()
        self.cur_pos = (0.0, 0.0)
        self.cur_ch = 1
        self.vt = 0.0
        self.seq = 0
        self.n_action = 0
        self.start_wall = time.time()
        self.remaining_real = 1200.0
        self.log_lines = []
        self.log_path = log_path
        self._cur_dets = []

    # ---------- 基础工具 ----------
    def log(self, msg):
        line = f"[vt={self.vt:8.1f}s pos=({self.cur_pos[0]:7.1f},{self.cur_pos[1]:7.1f})] {msg}"
        self.log_lines.append(line)
        if self.verbose:
            print(line)

    def _rid(self, tag):
        self.seq += 1
        return f"{tag}-{self.seq}"

    def _time_left(self):
        return self.remaining_real - (time.time() - self.start_wall)

    # ---------- 四个动作 ----------
    def enter(self):
        r = self.c.enter("enter-1")
        assert r.get("accepted") is True, f"/enter 未被接受: {r}"
        self.remaining_real = float(r.get("remaining_real_duration_s", 1200))
        self.start_wall = time.time()
        self.log(f"/enter 成功，本局现实可用 {self.remaining_real:.0f}s；"
                 f"{N_STATIONS} 站位环半径 {PLAN_R:.0f}，覆盖最坏距离 "
                 f"{coverage_worst_distance(500):.1f}m（<{EFF_R_MIN:.0f}）")

    def exit(self):
        try:
            r = self.c.exit(self._rid("exit"))
            self.log(f"/exit -> {r.get('exit_reason', r)}")
        except Exception as e:
            self.log(f"/exit 异常（测试可能已结束）：{e}")
        if self.log_path:
            with open(self.log_path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.log_lines))

    def measure(self, x, y, ch):
        """移动到(x,y)检测频道ch，更新跟踪状态。返回 measure_result 字符串。"""
        x = float(x); y = float(y)
        r = self.c.measure(self._rid("measure"), x, y, int(ch))
        if r.get("accepted") is not True:
            raise RuntimeError(f"/measure 未被接受: {r}")
        self.n_action += 1
        self.vt = float(r.get("virtual_time_s", self.vt))
        self.cur_pos = (x, y)
        self.cur_ch = int(ch)
        res = r.get("measure_result")
        tr = self.tracks.setdefault(int(ch), Track(int(ch)))
        if res == "direction":
            tr.dets.append((x, y, float(r["svd_deg"])))
            tr.probe = False
            self._refresh_region(tr)
            self.log(f"检测 ch{ch} @({x:.0f},{y:.0f}) 示向度 {r['svd_deg']:.2f}° "
                     f"（第{len(tr.dets)}次）-> {self._region_desc(tr)}")
        elif res == "near":
            tr.near = True
            tr.state = "tight"
            tr.target = (x, y)
            tr.mec = (x, y, 0.0)
            tr.diam = 0.0
            self.log(f"检测 ch{ch} @({x:.0f},{y:.0f}) 距离过近near，就地可清除")
        else:
            tr.tried.add(Track._key((x, y)))
            if not tr.dets:        # 从未测到示向度：不构成跟踪，留给后续站位复扫
                self.tracks.pop(int(ch), None)
            self.log(f"检测 ch{ch} @({x:.0f},{y:.0f}) 无信号")
        return res

    def clear(self, tr):
        tx, ty = tr.target
        r = self.c.clear(self._rid("clear"), tx, ty, tr.ch)
        if r.get("accepted") is not True:
            raise RuntimeError(f"/clear 未被接受: {r}")
        self.n_action += 1
        self.vt = float(r.get("virtual_time_s", self.vt))
        self.cur_pos = (tx, ty)
        if r.get("clear_result") == "success":
            self.cleared.add(tr.ch)
            self.tracks.pop(tr.ch, None)
            self.log(f"★ 清除 ch{tr.ch} 成功 @({tx:.1f},{ty:.1f})，累计清除 {len(self.cleared)} 个")
            return True
        else:
            tr.clear_attempts += 1
            tr.state = "open"
            tr.probe = True          # 回到当前位置原地补测，重新收敛
            self.log(f"清除 ch{tr.ch} 未发现（第{tr.clear_attempts}次），原地补测重新定位")
            return False

    # ---------- T1 交会区域计算 ----------
    def _refresh_region(self, tr):
        """用全部示向度重算交会多边形。可行域是半平面交集，有界后只随检测增加而缩小。"""
        if len(tr.dets) < 2:
            tr.bounded, tr.state = False, "open"
            return
        verts = feasible_vertices(tr.dets)
        hull = T1.convex_hull(verts)
        if len(hull) < 3:  # 本次无界/退化：若曾有界则沿用上次区域（新楔形只是没起作用）
            if tr.last_mec is not None:
                tr.bounded = True
                tr.mec = tr.last_mec
                tr.diam = tr.last_diam
                tr.state = "tight" if (tr.last_diam <= DIAM_LIMIT and
                                       tr.mec[2] <= SAFE_CLEAR_R) else "open"
            else:
                tr.bounded, tr.state = False, "open"
            tr.hull = hull
            return
        diam, (p1, p2) = T1.polygon_diameter(hull)
        contained, _, dr, _ = T1.check_polygon_in_diameter_circle(hull)
        cx, cy, mr = minimal_enclosing_circle([(p.x, p.y) for p in hull])
        tr.hull, tr.bounded = hull, True
        tr.diam, tr.diam_contained, tr.mec = diam, contained, (cx, cy, mr)
        # 收敛停滞计数：直径没有降到上次的 85% 记一次
        if tr.last_diam is not None and diam >= 0.85 * tr.last_diam - 1e-9:
            tr.stall += 1
        else:
            tr.stall = 0
        tr.last_mec, tr.last_diam = (cx, cy, mr), diam
        # 清除瞄准点：直径圆能包住且半径够小用直径圆圆心，否则用最小外接圆圆心
        if contained and dr <= SAFE_CLEAR_R:
            tr.target, tr.target_r = ((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0), dr
        else:
            tr.target, tr.target_r = (cx, cy), mr
        tr.state = "tight" if (diam <= DIAM_LIMIT and tr.target_r <= SAFE_CLEAR_R) else "open"

    @staticmethod
    def _region_desc(tr):
        if not tr.bounded:
            return "区域无界，需补测"
        return (f"直径{tr.diam:.1f} 外接R{tr.mec[2]:.1f} "
                f"{'锁定' if tr.state == 'tight' else '需补测'}")

    # ---------- 候选检测点 ----------
    def _valid(self, q, tr, lo=MIN_BASELINE):
        if not (math.isfinite(q[0]) and math.isfinite(q[1])):
            return False
        if abs(q[0]) > MAX_COORD or abs(q[1]) > MAX_COORD:
            return False
        if Track._key(q) in tr.tried:
            return False
        for (dx, dy, _) in tr.dets:
            if dist(q, (dx, dy)) < lo:          # 保证交会基线
                return False
        return True

    def second_candidates(self, tr):
        """第二检测点：以首次检测点为已知点套用 T2 良好区，并偏好“前距适中、横向拉开”的点。"""
        x0, y0, d0 = tr.dets[0]
        th = math.radians(d0)
        ex, ey = math.cos(th), math.sin(th)         # 示向方向
        nx, ny = -math.sin(th), math.cos(th)        # 左侧法向
        _, good = T2_apply.compute_positions(x0, y0, d0)
        out = []
        for q in good:
            if not self._valid(q, tr):
                continue
            fx, fy = q[0] - x0, q[1] - y0
            ahead = fx * ex + fy * ey
            lateral = fx * nx + fy * ny
            if 420.0 <= ahead <= 820.0 and 200.0 <= abs(lateral) <= 680.0:
                out.append(q)
        return out or [q for q in good if self._valid(q, tr)]

    def ring_candidates(self, tr):
        """三次及以上：围绕最近有界区域圆心，自适应半径取环，保证能收到信号。"""
        cx, cy, rr = tr.last_mec
        hi = max(120.0, RING_GUARD - rr)            # |Q-G|≤半径+rr<1000，必能收到
        base_radii = (RING_R, 450.0, 650.0)
        if len(tr.dets) >= 7:                       # 多次仍不收敛：贴近区域强收敛
            base_radii = (200.0, 300.0, RING_R)
        radii = []
        for rad in base_radii:
            r = min(rad, hi)
            if r >= 120.0 and r not in radii:
                radii.append(r)
        out = []
        for radius in radii:
            for k in range(32):
                a = 2 * math.pi * k / 32.0
                q = (cx + radius * math.cos(a), cy + radius * math.sin(a))
                if self._valid(q, tr, lo=150.0):
                    out.append(q)
        return out

    def _conditioning(self, center, q):
        """Q 处视线与各已有检测点视线在区域中心处交会角的最小 |sin|，越大越接近正交。"""
        cx, cy = center[0], center[1]
        v0 = (q[0] - cx, q[1] - cy)
        n0 = math.hypot(*v0)
        if n0 < 1e-9:
            return 0.0
        worst = 1.0
        for (x, y, _) in self._cur_dets:
            v1 = (cx - x, cy - y)
            n1 = math.hypot(*v1)
            if n1 < 1e-9:
                continue
            worst = min(worst, abs(v0[0] * v1[1] - v0[1] * v1[0]) / (n0 * n1))
        return worst

    def best_lock_point(self, tr, anchor):
        """挑选补测点：几何交会角优先（保证快速收敛），同级再取最顺路；停滞时纯几何逃逸。"""
        self._cur_dets = tr.dets
        if not tr.bounded or tr.last_mec is None:
            cands = self.second_candidates(tr)
            if not cands:                                    # 兜底：沿最新示向方向前进 600m
                x, y, d = tr.dets[-1]
                a = math.radians(d)
                return (x + 600.0 * math.cos(a), y + 600.0 * math.sin(a))

            def route_cost(q):
                c = dist(self.cur_pos, q)
                return c + (0.5 * dist(q, anchor) if anchor is not None else 0.0)

            return min(cands, key=route_cost)

        center = (tr.last_mec[0], tr.last_mec[1])
        cands = self.ring_candidates(tr)
        if not cands:                                        # 兜底：放宽到 T2 区
            cands = self.second_candidates(tr)
        if not cands:                                        # 最终兜底：贴近区域中心
            return center
        scored = [(self._conditioning(center, q), q) for q in cands]
        good = [t for t in scored if t[0] >= GOOD_SIN]       # 交会角≥35°
        if tr.stall >= 2 or not good:                        # 停滞/无好角：纯几何最优逃逸
            return max(scored, key=lambda t: t[0])[1]

        def route_cost(t):
            q = t[1]
            c = dist(self.cur_pos, q)
            return c + (0.5 * dist(q, anchor) if anchor is not None else 0.0)

        return min(good, key=route_cost)[1]

    # ---------- 站位批量扫描 ----------
    def sweep_channels_at_station(self):
        st = self.stations[self.station_idx]

        def _scan_needed(c):
            if c in self.cleared:
                return False
            tr = self.tracks.get(c)
            return (tr is None) or (tr.state != "tight")

        chs = [c for c in ALL_CHANNELS if _scan_needed(c)]
        # 从当前频道起升序环绕，省一次频道切换
        chs = sorted(set(chs), key=lambda c: ((c - self.cur_ch) % 20, c))
        self.log(f"==== 前往扫描站位 #{self.station_idx + 1}/{N_STATIONS} {st[0]:.0f},{st[1]:.0f}），"
                 f"待扫频道 {chs}")
        for ch in chs:
            self.measure(st[0], st[1], ch)
            if self._time_left() < RESERVE_EXIT_S:
                return
        self.station_idx += 1

    # ---------- 决策：贪心（阈值触发+最近邻），多目标用 NN+2-opt ----------
    def choose_action(self):
        """返回 (kind, payload)。kind: clear / lock / sweep / done。"""
        # 0) 清除失败后的原地补测，零移动优先
        for tr in self.tracks.values():
            if tr.probe:
                return "lock", (tr, self.cur_pos)
            if tr.clear_attempts >= 4:        # 保护：放弃反复清除失败的频道，交给站位复扫
                tr.state = "open"
                tr.probe = False

        # 1) 已锁定：NN+2-opt 排序后取首个清除
        tight = [tr for tr in self.tracks.values() if tr.state == "tight"]
        if tight:
            if len(tight) == 1:
                return "clear", (tight[0], tight[0].target)
            order = nn_then_2opt(self.cur_pos, [(id(tr), tr.target) for tr in tight])
            idmap = {id(tr): tr for tr in tight}
            first = idmap[order[0]]
            return "clear", (first, first.target)

        # 2) 未锁定：计算各跟踪的最佳补测点
        anchor = self.stations[self.station_idx] if self.station_idx < N_STATIONS else None
        lock_opts = []
        for tr in self.tracks.values():
            if tr.state != "open" or not tr.dets:
                continue
            q = self.best_lock_point(tr, anchor)
            if q is None:
                continue
            dcur = dist(self.cur_pos, q)
            if anchor is not None:
                detour = dcur + dist(q, anchor) - dist(self.cur_pos, anchor)
            else:
                detour = 0.0
            lock_opts.append((dcur, detour, tr, q))

        # 2a) 阈值触发：顺路 或 就近，则最近邻插队
        onway = [o for o in lock_opts if o[1] <= DETOUR_TOL or o[0] <= NEAR_TOL]
        if onway:
            onway.sort(key=lambda o: o[0])
            _, _, tr, q = onway[0]
            return "lock", (tr, q)

        # 3) 还有站位未扫：推进覆盖主干
        if self.station_idx < N_STATIONS:
            return "sweep", None

        # 4) 站位已扫完：必须把剩余 open 跟踪逐个收敛清除
        if lock_opts:
            lock_opts.sort(key=lambda o: o[0])
            _, _, tr, q = lock_opts[0]
            return "lock", (tr, q)

        return "done", None

    # ---------- 主循环 ----------
    def run(self):
        info = None
        self.enter()
        try:
            while self.n_action < MAX_ACTIONS:
                if self._time_left() < RESERVE_EXIT_S:
                    self.log("现实时间预留到达，准备收尾退出。")
                    break
                kind, payload = self.choose_action()
                if kind == "done":
                    self.log("全部跟踪已清除，且 7 站位扫描完毕，任务完成。")
                    break
                if kind == "sweep":
                    self.sweep_channels_at_station()
                elif kind == "clear":
                    tr, _ = payload
                    self.clear(tr)
                elif kind == "lock":
                    tr, q = payload
                    self.measure(q[0], q[1], tr.ch)
            info = self.summary()
        finally:
            self.exit()
        return info

    def summary(self):
        total = self.vt
        n = len(self.cleared)
        avg = total / n if n else 0.0
        info = {
            "cleared_channels": sorted(self.cleared),
            "n_cleared": n,
            "virtual_time_s": round(total, 2),
            "avg_time_per_clear_s": round(avg, 2),
            "n_actions": self.n_action,
            "stations_visited": min(self.station_idx, N_STATIONS),
        }
        self.log("统计：" + json.dumps(info, ensure_ascii=False))
        return info


# ============================================================================
# 五、入口
# ============================================================================
def run_real(log_path="T3_run_log.txt"):
    if ROBOT_ID.strip() in ("", "<参赛队号>"):
        raise SystemExit("请先在顶层参数 ROBOT_ID 填入当前模拟器登录的参赛队号。")
    mis = Mission(RealSimClient(), verbose=True, log_path=log_path)
    return mis.run()


def run_self_test(cases=1, verbose=False):
    succ = 0
    times, actions, counts = [], [], []
    for i in range(cases):
        cli = MockSimClient(seed=20260913 + i)
        n_true = len(cli.sources)
        mis = Mission(cli, verbose=verbose)
        info = mis.run()
        ok = (info["n_cleared"] == n_true) and (set(cli.sources) == cli.cleared)
        if ok:
            succ += 1
        times.append(info["virtual_time_s"]); actions.append(info["n_actions"]); counts.append(n_true)
        print(f"案例{i+1:03d}: 真源{n_true:2d} 清除{info['n_cleared']:2d} "
              f"{'成功' if ok else '★失败★'} 虚拟总时{info['virtual_time_s']:.0f}s "
              f"平均{info['avg_time_per_clear_s']:.0f}s/个 动作{info['n_actions']}")
    print("-" * 70)
    print(f"成功率 {succ}/{cases} = {succ/cases*100:.1f}%；"
          f"平均虚拟总时 {sum(times)/len(times):.0f}s，平均动作 {sum(actions)/len(actions):.0f}")
    return succ == cases


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true", help="离线随机自测，不连接模拟器")
    ap.add_argument("-n", "--cases", type=int, default=1, help="自测随机案例数")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印每个动作")
    args = ap.parse_args()
    if args.self_test:
        run_self_test(args.cases, verbose=args.verbose)
    else:
        print(run_real())
