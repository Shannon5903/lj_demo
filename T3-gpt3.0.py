# -*- coding: utf-8 -*-
"""第三问：滚动规划的动态测向与清除调度器。

依赖：将本文件与前两问的 T1.py、T2.py 放在同一目录。

设计边界：
1. 本文件不重新实现前两问的交会区域、凸包、直径或第二监测区域评分；
2. GeometryAdapter 只负责调用 T1，SecondRegionProvider 只负责调用/读取 T2；
3. 调度层统一比较 MEASURE_BATCH 与 CLEAR 两类 Action；
4. 默认 mock 模式可离线验证，http 模式直接连接题目模拟器。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import time
import uuid
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Protocol, Sequence, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import T1  # 第一问已有模块
import T2  # 第二问已有模块


# ============================================================================
# 集中参数区：后续敏感性分析只需要修改此处
# ============================================================================


@dataclass
class PlannerConfig:
    # 题目给定时间参数
    robot_speed_mps: float = 5.0
    measure_time_s: float = 5.0
    channel_switch_time_s: float = 1.0
    optical_time_s: float = 3.0
    clear_extra_time_s: float = 2.0

    # 定位、搜索区域与频道
    diameter_limit_m: float = 40.0
    clear_radius_m: float = 20.0
    channel_min: int = 1
    channel_max: int = 20
    expected_source_min: int = 10
    expected_source_max: int = 16
    arena_center_x: float = 0.0
    arena_center_y: float = 0.0
    arena_radius_m: float = 1800.0

    # 中心首扫与外围路标
    outer_waypoint_radius_m: float = 1000.0
    outer_waypoint_count: int = 6
    outer_start_angle_deg: float = 0.0

    # 第二问候选区域离散化
    max_samples_per_region: int = 90
    candidate_merge_grid_m: float = 25.0
    revisit_tolerance_m: float = 8.0
    refinement_radius_m: float = 400.0
    refinement_angle_count: int = 8
    refinement_match_tolerance_m: float = 40.0
    focused_refinement_after_observations: int = 2
    repeated_measure_penalty: float = 0.45

    # 动作评分权重
    w_information_early: float = 18.0
    w_information_late: float = 7.0
    w_clear_early: float = 5.0
    w_clear_late: float = 20.0
    w_discovery: float = 8.0
    w_overlap_bonus: float = 0.30
    w_on_route_clear: float = 2.5
    w_served: float = 1.0
    w_expected_locatable: float = 3.0
    route_detour_penalty_per_m: float = 0.0002
    radial_backtrack_penalty_per_m: float = 0.0002
    allowed_radial_backtrack_m: float = 300.0
    max_measure_route_detour_m: float = 650.0
    min_measure_utility_before_waypoint: float = 0.015
    immediate_clear_distance_m: float = 60.0
    clear_on_route_threshold_m: float = 300.0
    force_on_route_clear: bool = True

    # 运行与输出控制
    max_planning_steps: int = 500
    print_top_candidates: int = 8
    random_seed: int = 2026


# ============================================================================
# 基础数据结构
# ============================================================================


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def distance_to(self, other: "Vec2") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_dict(self) -> Dict[str, float]:
        return {"x": round(self.x, 6), "y": round(self.y, 6)}


class ChannelStatus(str, Enum):
    UNKNOWN = "UNKNOWN"        # 尚未获得有效示向度，可能无源，也可能暂时收不到
    UNLOCATED = "UNLOCATED"    # 已有示向度，但定位直径仍大于 40 m 或仅有一次测向
    LOCATABLE = "LOCATABLE"    # 定位直径不大于 40 m，可进行光学搜索和清除
    CLEARED = "CLEARED"        # 已成功清除


@dataclass
class BearingObservation:
    position: Vec2
    direction_deg: float
    virtual_time_s: float


@dataclass
class ChannelTask:
    channel: int
    status: ChannelStatus = ChannelStatus.UNKNOWN
    observations: List[BearingObservation] = field(default_factory=list)
    second_region: Optional["CandidateRegion"] = None
    polygon: List[Vec2] = field(default_factory=list)
    diameter_m: Optional[float] = None
    clear_position: Optional[Vec2] = None
    measured_positions: List[Vec2] = field(default_factory=list)
    no_signal_count: int = 0
    clear_failures: int = 0


class ActionType(str, Enum):
    CENTRAL_SCAN = "CENTRAL_SCAN"
    OUTER_SCAN = "OUTER_SCAN"
    MEASURE_BATCH = "MEASURE_BATCH"
    CLEAR = "CLEAR"


@dataclass
class Action:
    kind: ActionType
    position: Vec2
    channels: Tuple[int, ...]
    utility: float = 0.0
    estimated_time_s: float = 0.0
    move_distance_m: float = 0.0
    information_count: int = 0
    route_detour_m: float = 0.0
    radial_backtrack_m: float = 0.0
    details: Dict[str, float] = field(default_factory=dict)

    def short(self) -> str:
        return (
            f"{self.kind.value} P=({self.position.x:.1f},{self.position.y:.1f}) "
            f"channels={list(self.channels)} J={self.utility:.5f} "
            f"dt={self.estimated_time_s:.2f}s"
        )


@dataclass
class StepRecord:
    step: int
    action: Dict[str, object]
    start_position: Dict[str, float]
    end_position: Dict[str, float]
    uncleared_detected_channels: int
    locatable_channels: int
    candidate_actions: List[Dict[str, object]]
    move_distance_m: float
    estimated_step_time_s: float
    actual_virtual_time_s: float
    cumulative_move_distance_m: float
    cumulative_measurements: int
    cumulative_clears: int


@dataclass
class MissionState:
    position: Vec2
    channels: Dict[int, ChannelTask]
    receiver_channel: int = 1
    virtual_time_s: float = 0.0
    total_move_distance_m: float = 0.0
    total_measurements: int = 0
    total_clear_attempts: int = 0
    successful_clears: int = 0
    central_scan_done: bool = False
    outer_waypoint_index: int = 0
    radial_frontier_m: float = 0.0
    trajectory: List[Vec2] = field(default_factory=list)
    monitoring_sequence: List[Dict[str, object]] = field(default_factory=list)
    clear_order: List[int] = field(default_factory=list)
    records: List[StepRecord] = field(default_factory=list)


# ============================================================================
# 前两问适配层
# ============================================================================


@dataclass
class LocalizationResult:
    polygon: List[Vec2]
    diameter_m: float
    diameter_midpoint: Vec2


class GeometryAdapter:
    """只调用第一问 T1.py 中已有的几何函数。"""

    @staticmethod
    def localize(observations: Sequence[BearingObservation]) -> Optional[LocalizationResult]:
        if len(observations) < 2:
            return None

        # T1 的标准算例以第一监测点位于原点、方向为 90° 表示。
        # 这里只做刚体变换，不改变 T1 的任何几何计算。
        anchor = observations[0]
        rotate_deg = 90.0 - anchor.direction_deg
        rotate_rad = math.radians(rotate_deg)
        c, s = math.cos(rotate_rad), math.sin(rotate_rad)

        def to_standard(p: Vec2) -> Vec2:
            dx, dy = p.x - anchor.position.x, p.y - anchor.position.y
            return Vec2(c * dx - s * dy, s * dx + c * dy)

        def to_global(p: Vec2) -> Vec2:
            # 标准坐标逆时针旋转 -rotate_deg，再平移回 anchor。
            return Vec2(
                anchor.position.x + c * p.x + s * p.y,
                anchor.position.y - s * p.x + c * p.y,
            )

        detection_points: List[Tuple[float, float, float]] = []
        for o in observations:
            p = to_standard(o.position)
            detection_points.append((p.x, p.y, (o.direction_deg + rotate_deg) % 360.0))

        def solve(points: Sequence[Tuple[float, float, float]]) -> Optional[LocalizationResult]:
            vertices = T1.compute_feasible_vertices(list(points))
            if len(vertices) < 2:
                return None
            hull = T1.convex_hull(vertices)
            if len(hull) < 2:
                return None
            diameter, pair = T1.polygon_diameter(hull)
            p1, p2 = pair
            if p1 is None or p2 is None:
                return None
            midpoint = to_global(Vec2((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0))
            return LocalizationResult(
                polygon=[to_global(Vec2(p.x, p.y)) for p in hull],
                diameter_m=float(diameter),
                diameter_midpoint=midpoint,
            )

        # 优先使用全部观测；同时检查各两测向组合。
        # 这样既调用原 T1 算法，又能避免某次临界角/数值退化使整个交集返回空集。
        candidates: List[LocalizationResult] = []
        all_result = solve(detection_points)
        if all_result is not None:
            candidates.append(all_result)
        for i in range(len(detection_points)):
            for j in range(i + 1, len(detection_points)):
                pair_result = solve([detection_points[i], detection_points[j]])
                if pair_result is not None:
                    candidates.append(pair_result)
        return min(candidates, key=lambda r: r.diameter_m, default=None)


def _point_in_polygon(point: Vec2, polygon: Sequence[Vec2]) -> bool:
    """调度层用于判断候选点是否属于第二问输出区域。"""
    if len(polygon) < 3:
        return False
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        a, b = polygon[i], polygon[j]
        cross = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x)
        if abs(cross) <= 1e-8 and (
            min(a.x, b.x) - 1e-8 <= point.x <= max(a.x, b.x) + 1e-8
            and min(a.y, b.y) - 1e-8 <= point.y <= max(a.y, b.y) + 1e-8
        ):
            return True
        if (a.y > point.y) != (b.y > point.y):
            x_hit = (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
            if point.x < x_hit:
                inside = not inside
        j = i
    return inside


@dataclass
class StandardSecondRegion:
    polygons: List[List[Vec2]]
    samples: List[Vec2]


@dataclass
class CandidateRegion:
    """第二问标准区域经平移、旋转后的刚体变换视图。"""

    origin: Vec2
    bearing_deg: float
    standard: StandardSecondRegion

    def _to_global(self, local: Vec2) -> Vec2:
        th = math.radians(self.bearing_deg)
        # 标准坐标：+y 为前向，+x 为中心线右侧
        right = Vec2(math.sin(th), -math.cos(th))
        forward = Vec2(math.cos(th), math.sin(th))
        return Vec2(
            self.origin.x + local.x * right.x + local.y * forward.x,
            self.origin.y + local.x * right.y + local.y * forward.y,
        )

    def _to_local(self, global_point: Vec2) -> Vec2:
        th = math.radians(self.bearing_deg)
        dx = global_point.x - self.origin.x
        dy = global_point.y - self.origin.y
        right = Vec2(math.sin(th), -math.cos(th))
        forward = Vec2(math.cos(th), math.sin(th))
        return Vec2(dx * right.x + dy * right.y, dx * forward.x + dy * forward.y)

    def contains(self, point: Vec2) -> bool:
        local = self._to_local(point)
        return any(_point_in_polygon(local, poly) for poly in self.standard.polygons)

    def candidate_samples(self) -> List[Vec2]:
        return [self._to_global(p) for p in self.standard.samples]


class SecondRegionProvider:
    """读取 T2 的结果；若结果文件不存在，则在 /enter 前调用 T2 生成一次。"""

    def __init__(self, cfg: PlannerConfig):
        self.cfg = cfg
        self.standard = self._load_or_build_standard_region()

    def _load_or_build_points(self) -> List[Tuple[float, float]]:
        result_path = Path(getattr(T2, "RESULT_FILE", "t2_results.json"))
        if result_path.exists():
            data = json.loads(result_path.read_text(encoding="utf-8"))
            points = [tuple(p) for p in data.get("optimal", []) + data.get("good", [])]
            if points:
                return [(float(x), float(y)) for x, y in points]

        # 第二问函数只在此调用一次；应在正式 /enter 之前完成，避免占用现实测试时间。
        targets = T2.enumerate_targets(T2.N_TARGETS)
        triangle = T2.known_triangle_vertices()
        results = T2.compute_scores(targets, triangle, verbose=False)
        optimal, good, _, _ = T2.classify(results, T2.N_TARGETS)
        points = optimal + good
        if not points:
            raise RuntimeError("T2 未生成最优/良好第二监测位置")
        return [(float(x), float(y)) for x, y in points]

    @staticmethod
    def _hull(points: Sequence[Tuple[float, float]]) -> List[Vec2]:
        t1_points = [T1.Point2D(x, y) for x, y in points]
        return [Vec2(p.x, p.y) for p in T1.convex_hull(t1_points)]

    def _load_or_build_standard_region(self) -> StandardSecondRegion:
        points = self._load_or_build_points()
        # 第二问结果通常关于 y 轴形成左右两个高分区域，分别取凸包，避免把中间低分带填满。
        left = [(x, y) for x, y in points if x <= 0]
        right = [(x, y) for x, y in points if x >= 0]
        polygons = [self._hull(side) for side in (left, right) if len(side) >= 3]
        if not polygons:
            polygons = [self._hull(points)]

        ordered = sorted((Vec2(x, y) for x, y in points), key=lambda p: (p.y, p.x))
        stride = max(1, math.ceil(len(ordered) / self.cfg.max_samples_per_region))
        samples = ordered[::stride]
        # 加入每个区域质心，使重叠区域内部也有代表候选点。
        for poly in polygons:
            samples.append(Vec2(
                sum(p.x for p in poly) / len(poly),
                sum(p.y for p in poly) / len(poly),
            ))
        return StandardSecondRegion(polygons=polygons, samples=samples)

    def for_first_observation(self, obs: BearingObservation) -> CandidateRegion:
        return CandidateRegion(obs.position, obs.direction_deg, self.standard)


# ============================================================================
# 模拟器接口与离线测试接口
# ============================================================================


class RobotClient(Protocol):
    def enter(self) -> Dict[str, object]: ...
    def measure(self, position: Vec2, channel: int) -> Dict[str, object]: ...
    def clear(self, position: Vec2, channel: int) -> Dict[str, object]: ...
    def exit(self) -> Dict[str, object]: ...


class HttpRobotClient:
    def __init__(self, base_url: str, robot_id: str, timeout_s: float = 5.0):
        if not robot_id or robot_id.startswith("<"):
            raise ValueError("http 模式必须提供真实参赛队号 --robot-id")
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.timeout_s = timeout_s
        self.counter = 0

    def _request_id(self, prefix: str) -> str:
        self.counter += 1
        return f"{prefix}-{self.counter}-{uuid.uuid4().hex[:8]}"

    def _base(self, request_id: str) -> Dict[str, object]:
        return {"arena_id": "default", "robot_id": self.robot_id, "request_id": request_id}

    def _post(self, path: str, payload: Dict[str, object]) -> Dict[str, object]:
        body = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        last_error: Optional[Exception] = None
        # 网络异常时复用完全相同的 payload 和 request_id，符合幂等规则。
        for attempt in range(3):
            try:
                request = Request(
                    self.base_url + path,
                    data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urlopen(request, timeout=self.timeout_s) as response:
                    data = json.loads(response.read().decode("utf-8"))
                    if response.status != 200:
                        raise RuntimeError(f"HTTP {response.status}: {data}")
                    if data.get("accepted") is not True:
                        raise RuntimeError(f"动作未被接受: {data}")
                    return data
            except HTTPError as exc:
                try:
                    text = exc.read().decode("utf-8")
                except Exception:
                    text = ""
                raise RuntimeError(f"HTTP {exc.code}: {text}") from exc
            except (URLError, TimeoutError, OSError) as exc:
                last_error = exc
                if attempt < 2:
                    time.sleep(0.25 * (2 ** attempt))
        raise RuntimeError(f"连接模拟器失败: {last_error}")

    def enter(self) -> Dict[str, object]:
        rid = self._request_id("enter")
        return self._post("/enter", self._base(rid))

    def measure(self, position: Vec2, channel: int) -> Dict[str, object]:
        rid = self._request_id("measure")
        payload = self._base(rid)
        payload.update({"position": position.to_dict(), "channel": int(channel)})
        return self._post("/measure", payload)

    def clear(self, position: Vec2, channel: int) -> Dict[str, object]:
        rid = self._request_id("clear")
        payload = self._base(rid)
        payload.update({"position": position.to_dict(), "channel": int(channel)})
        return self._post("/clear", payload)

    def exit(self) -> Dict[str, object]:
        rid = self._request_id("exit")
        return self._post("/exit", self._base(rid))


@dataclass
class MockSource:
    channel: int
    position: Vec2
    receive_radius_m: float = 1500.0
    cleared: bool = False


class MockRobotClient:
    """离线接口：用于验证调度闭环，不替代正式模拟器。"""

    def __init__(self, sources: Sequence[MockSource], cfg: PlannerConfig):
        self.sources = {s.channel: s for s in sources}
        self.cfg = cfg
        self.position = Vec2(0.0, 0.0)
        self.receiver_channel = 1
        self.virtual_time_s = 0.0
        self.rng = random.Random(cfg.random_seed)

    def enter(self) -> Dict[str, object]:
        return {"accepted": True, "virtual_time_s": self.virtual_time_s,
                "remaining_real_duration_s": 1200}

    def _move(self, p: Vec2) -> None:
        self.virtual_time_s += self.position.distance_to(p) / self.cfg.robot_speed_mps
        self.position = p

    def measure(self, position: Vec2, channel: int) -> Dict[str, object]:
        self._move(position)
        if channel != self.receiver_channel:
            self.virtual_time_s += self.cfg.channel_switch_time_s
        self.receiver_channel = channel
        self.virtual_time_s += self.cfg.measure_time_s
        source = self.sources.get(channel)
        if source is None or source.cleared or position.distance_to(source.position) > source.receive_radius_m:
            return {"accepted": True, "virtual_time_s": self.virtual_time_s,
                    "measure_result": "no_signal"}
        distance = position.distance_to(source.position)
        if distance <= 5.0:
            return {"accepted": True, "virtual_time_s": self.virtual_time_s,
                    "measure_result": "near"}
        true_angle = math.degrees(math.atan2(
            source.position.y - position.y, source.position.x - position.x
        )) % 360.0
        observed = (true_angle + self.rng.uniform(-1.0, 1.0)) % 360.0
        return {"accepted": True, "virtual_time_s": self.virtual_time_s,
                "measure_result": "direction", "svd_deg": round(observed, 2)}

    def clear(self, position: Vec2, channel: int) -> Dict[str, object]:
        self._move(position)
        source = self.sources.get(channel)
        success = bool(
            source and not source.cleared
            and position.distance_to(source.position) <= self.cfg.clear_radius_m
        )
        if success:
            source.cleared = True
            self.virtual_time_s += self.cfg.optical_time_s + self.cfg.clear_extra_time_s
            result = "success"
        else:
            self.virtual_time_s += self.cfg.optical_time_s
            result = "no_target_in_range"
        return {"accepted": True, "virtual_time_s": self.virtual_time_s,
                "clear_result": result}

    def exit(self) -> Dict[str, object]:
        return {"accepted": True, "virtual_time_s": self.virtual_time_s,
                "exit_reason": "user_exit"}


# ============================================================================
# 第三问调度器
# ============================================================================


class RollingPlanner:
    def __init__(
        self,
        client: RobotClient,
        cfg: PlannerConfig,
        region_provider: SecondRegionProvider,
        log_path: Path,
    ):
        self.client = client
        self.cfg = cfg
        self.region_provider = region_provider
        self.log_path = log_path
        channels = {
            ch: ChannelTask(channel=ch)
            for ch in range(cfg.channel_min, cfg.channel_max + 1)
        }
        self.state = MissionState(
            position=Vec2(0.0, 0.0),
            channels=channels,
            trajectory=[Vec2(0.0, 0.0)],
        )
        self.outer_waypoints = self._make_outer_waypoints()

    # ---------- 中心向外的宏观路线 ----------

    def _make_outer_waypoints(self) -> List[Vec2]:
        """生成半径约 1000 m 的六个外围路标，顺序固定但不作为扇区约束。"""
        result = []
        for k in range(self.cfg.outer_waypoint_count):
            angle = (
                self.cfg.outer_start_angle_deg
                + 360.0 * k / self.cfg.outer_waypoint_count
            )
            th = math.radians(angle)
            result.append(Vec2(
                self.cfg.arena_center_x
                + self.cfg.outer_waypoint_radius_m * math.cos(th),
                self.cfg.arena_center_y
                + self.cfg.outer_waypoint_radius_m * math.sin(th),
            ))
        return result

    def _next_waypoint(self) -> Optional[Vec2]:
        idx = self.state.outer_waypoint_index
        return self.outer_waypoints[idx] if idx < len(self.outer_waypoints) else None

    def _radius(self, p: Vec2) -> float:
        return math.hypot(
            p.x - self.cfg.arena_center_x,
            p.y - self.cfg.arena_center_y,
        )

    def _radial_backtrack(self, p: Vec2) -> float:
        return max(
            0.0,
            self.state.radial_frontier_m
            - self._radius(p)
            - self.cfg.allowed_radial_backtrack_m,
        )

    def _route_costs(self, p: Vec2) -> Tuple[float, float]:
        """返回候选点相对当前路标线段的绕行距离和超限径向回退。"""
        waypoint = self._next_waypoint()
        detour = self._detour_via(self.state.position, p, waypoint)
        return detour, self._radial_backtrack(p)

    # ---------- 时间与阶段权重 ----------

    def _ordered_channels(self, channels: Iterable[int]) -> Tuple[int, ...]:
        unique = sorted(set(channels))
        if self.state.receiver_channel in unique:
            unique.remove(self.state.receiver_channel)
            unique.insert(0, self.state.receiver_channel)
        return tuple(unique)

    def _measure_batch_time(self, p: Vec2, channels: Sequence[int]) -> Tuple[float, float]:
        distance = self.state.position.distance_to(p)
        t = distance / self.cfg.robot_speed_mps
        current = self.state.receiver_channel
        for ch in channels:
            if ch != current:
                t += self.cfg.channel_switch_time_s
            t += self.cfg.measure_time_s
            current = ch
        return distance, t

    def _phase_weights(self) -> Tuple[float, float]:
        detected = [t for t in self.state.channels.values() if t.observations]
        if not detected:
            return self.cfg.w_information_early, self.cfg.w_clear_early
        remaining = sum(t.status != ChannelStatus.CLEARED for t in detected)
        locatable = sum(t.status == ChannelStatus.LOCATABLE for t in detected)
        progress = self.state.successful_clears / max(1, len(detected))
        information_fraction = max(0.0, min(1.0, (remaining - locatable) / max(1, remaining)))
        late = max(progress, 1.0 - information_fraction)
        w_info = (1.0 - late) * self.cfg.w_information_early + late * self.cfg.w_information_late
        w_clear = (1.0 - late) * self.cfg.w_clear_early + late * self.cfg.w_clear_late
        return w_info, w_clear

    # ---------- 动作生成 ----------

    def _not_revisited(self, task: ChannelTask, point: Vec2) -> bool:
        return all(
            point.distance_to(old) > self.cfg.revisit_tolerance_m
            for old in task.measured_positions
        )

    def _merge_candidate_points(self, points: Iterable[Vec2]) -> List[Vec2]:
        grid = self.cfg.candidate_merge_grid_m
        buckets: Dict[Tuple[int, int], List[Vec2]] = {}
        for p in points:
            key = (round(p.x / grid), round(p.y / grid))
            buckets.setdefault(key, []).append(p)
        return [
            Vec2(sum(p.x for p in group) / len(group), sum(p.y for p in group) / len(group))
            for group in buckets.values()
        ]

    def _central_scan_action(self) -> Optional[Action]:
        """任务开始后的强制第一步：在原点集中扫描全部频道。"""
        if self.state.central_scan_done:
            return None
        point = Vec2(self.cfg.arena_center_x, self.cfg.arena_center_y)
        channels = self._ordered_channels(self.state.channels)
        distance, t = self._measure_batch_time(point, channels)
        return Action(
            kind=ActionType.CENTRAL_SCAN,
            position=point,
            channels=channels,
            utility=1.0e9,
            estimated_time_s=t,
            move_distance_m=distance,
            information_count=len(channels),
            details={"central_scan": 1.0},
        )

    def _outer_scan_action(self) -> Optional[Action]:
        """只生成当前下一个外围路标，并且只扫描 UNKNOWN 频道。"""
        point = self._next_waypoint()
        if point is None:
            return None
        channels = self._ordered_channels(
            ch for ch, task in self.state.channels.items()
            if task.status == ChannelStatus.UNKNOWN
        )
        if not channels:
            return None
        distance, t = self._measure_batch_time(point, channels)
        return Action(
            kind=ActionType.OUTER_SCAN,
            position=point,
            channels=channels,
            utility=self.cfg.w_discovery * len(channels) / max(t, 1e-9),
            estimated_time_s=t,
            move_distance_m=distance,
            information_count=len(channels),
            details={
                "outer_waypoint_index": float(self.state.outer_waypoint_index),
                "unknown_count": float(len(channels)),
            },
        )

    def _secondary_measure_actions(self) -> List[Action]:
        tasks = [
            t for t in self.state.channels.values()
            if t.status == ChannelStatus.UNLOCATED and t.second_region is not None
        ]
        per_task_points: Dict[int, List[Vec2]] = {}
        refinement_points: Dict[int, List[Vec2]] = {}
        for task in tasks:
            points = list(task.second_region.candidate_samples())
            refinements: List[Vec2] = []
            # 若第二次测向后仍未达到 40 m，则围绕当前定位区域中心增加小范围复测点。
            # 半径 400 m 小于最小有效接收半径 1000 m，并可形成较大的交会角。
            if (
                len(task.observations) >= 2
                and task.diameter_m is not None
                and task.diameter_m > self.cfg.diameter_limit_m
                and task.clear_position is not None
            ):
                center = task.clear_position
                for k in range(self.cfg.refinement_angle_count):
                    th = 2.0 * math.pi * k / self.cfg.refinement_angle_count
                    refinements.append(Vec2(
                        center.x + self.cfg.refinement_radius_m * math.cos(th),
                        center.y + self.cfg.refinement_radius_m * math.sin(th),
                    ))
                # 已完成二次测向后，不再沿原高分区域逐点试探，改用局部环形复测。
                points = refinements
            per_task_points[task.channel] = points
            refinement_points[task.channel] = refinements

        raw_points = [p for points in per_task_points.values() for p in points]
        candidate_points = self._merge_candidate_points(raw_points)
        w_info, _ = self._phase_weights()
        actions = []
        for point in candidate_points:
            served = []
            for task in tasks:
                if not self._not_revisited(task, point):
                    continue
                near_refinement_sample = any(
                    point.distance_to(sample) <= self.cfg.refinement_match_tolerance_m
                    for sample in refinement_points[task.channel]
                )
                in_second_region = bool(
                    not refinement_points[task.channel]
                    and task.second_region
                    and task.second_region.contains(point)
                )
                if near_refinement_sample or in_second_region:
                    served.append(task.channel)
            if not served:
                continue
            # 两次有效示向后进入单频道精定位，不再为重叠数量捎带重复测量。
            focused = [
                ch for ch in served
                if len(self.state.channels[ch].observations)
                >= self.cfg.focused_refinement_after_observations
            ]
            if focused:
                served = [min(
                    focused,
                    key=lambda ch: (
                        self.state.channels[ch].clear_position.distance_to(point)
                        if self.state.channels[ch].clear_position else math.inf
                    ),
                )]
            channels = self._ordered_channels(served)
            distance, t = self._measure_batch_time(point, channels)
            n = len(channels)
            expected_locatable = sum(
                len(self.state.channels[ch].observations) == 1 for ch in channels
            )
            detour, radial_backtrack = self._route_costs(point)
            if (
                self._next_waypoint() is not None
                and detour > self.cfg.max_measure_route_detour_m
            ):
                continue
            repeat_count = max(
                len(self.state.channels[ch].observations) for ch in channels
            )
            repeat_factor = self.cfg.repeated_measure_penalty ** max(
                0, repeat_count - self.cfg.focused_refinement_after_observations
            )
            information_value = (
                self.cfg.w_served * n
                + self.cfg.w_expected_locatable * expected_locatable
            )
            utility = repeat_factor * (
                w_info * information_value / max(t, 1e-9)
                + self.cfg.w_overlap_bonus * max(0, n - 1)
            )
            if self._next_waypoint() is not None:
                utility -= self.cfg.route_detour_penalty_per_m * detour
                utility -= (
                    self.cfg.radial_backtrack_penalty_per_m * radial_backtrack
                )
            actions.append(Action(
                kind=ActionType.MEASURE_BATCH,
                position=point,
                channels=channels,
                utility=utility,
                estimated_time_s=t,
                move_distance_m=distance,
                information_count=n,
                route_detour_m=detour,
                radial_backtrack_m=radial_backtrack,
                details={
                    "overlap_count": float(n),
                    "expected_locatable_count": float(expected_locatable),
                    "direction_observations": float(repeat_count),
                    "repeat_factor": repeat_factor,
                    "focused_refinement": float(bool(focused)),
                },
            ))
        return actions

    @staticmethod
    def _detour_via(start: Vec2, via: Vec2, end: Optional[Vec2]) -> float:
        if end is None:
            return start.distance_to(via)
        return start.distance_to(via) + via.distance_to(end) - start.distance_to(end)

    def _clear_actions(self) -> List[Action]:
        _, w_clear = self._phase_weights()
        actions = []
        waypoint = self._next_waypoint()
        for task in self.state.channels.values():
            if task.status != ChannelStatus.LOCATABLE or task.clear_position is None:
                continue
            point = task.clear_position
            distance = self.state.position.distance_to(point)
            t = distance / self.cfg.robot_speed_mps + self.cfg.optical_time_s + self.cfg.clear_extra_time_s
            detour = self._detour_via(self.state.position, point, waypoint)
            radial_backtrack = self._radial_backtrack(point)
            on_route_bonus = 0.0
            if waypoint and detour <= self.cfg.clear_on_route_threshold_m:
                on_route_bonus = self.cfg.w_on_route_clear * (
                    1.0 - detour / self.cfg.clear_on_route_threshold_m
                )
            utility = (
                w_clear / max(t, 1e-9)
                + on_route_bonus
            )
            if waypoint is not None:
                utility -= self.cfg.radial_backtrack_penalty_per_m * radial_backtrack
            actions.append(Action(
                kind=ActionType.CLEAR,
                position=point,
                channels=(task.channel,),
                utility=utility,
                estimated_time_s=t,
                move_distance_m=distance,
                route_detour_m=detour,
                radial_backtrack_m=radial_backtrack,
                details={
                    "detour_m": detour,
                    "on_route_bonus": on_route_bonus,
                    "immediate": float(distance <= self.cfg.immediate_clear_distance_m),
                },
            ))
        return actions

    def generate_actions(self) -> List[Action]:
        central = self._central_scan_action()
        if central is not None:
            return [central]

        secondary = self._secondary_measure_actions()
        outer = self._outer_scan_action()
        clears = self._clear_actions()
        all_actions = secondary + clears + ([outer] if outer else [])
        ranked = sorted(all_actions, key=lambda a: a.utility, reverse=True)

        # 1. near 或当前位置附近目标：立即清除。
        immediate = [
            a for a in clears
            if a.move_distance_m <= self.cfg.immediate_clear_distance_m
        ]
        if immediate:
            chosen = min(immediate, key=lambda a: a.estimated_time_s)
            return [chosen] + [a for a in ranked if a is not chosen]

        # 2. 前往下一路标时绕行不超过阈值：硬优先顺路清除。
        if self.cfg.force_on_route_clear and self._next_waypoint() is not None:
            on_route = [
                a for a in clears
                if a.route_detour_m <= self.cfg.clear_on_route_threshold_m
            ]
            if on_route:
                chosen = min(
                    on_route,
                    key=lambda a: (a.route_detour_m, a.estimated_time_s),
                )
                return [chosen] + [a for a in ranked if a is not chosen]

        # 3. 在不明显偏离当前向外路线时，执行高价值联合二次测向。
        good_secondary = [
            a for a in secondary
            if a.utility >= self.cfg.min_measure_utility_before_waypoint
        ]
        if good_secondary:
            chosen = max(good_secondary, key=lambda a: a.utility)
            return [chosen] + [a for a in ranked if a is not chosen]

        # 4. 没有更好的沿途任务时，推进到下一个外围路标。
        if outer is not None:
            return [outer] + [a for a in ranked if a is not outer]

        # 5. 外围扫描结束后，按局部效用处理全部遗留定位/清除任务。
        return ranked

    # ---------- 状态更新 ----------

    def _update_from_measurement(
        self,
        task: ChannelTask,
        position: Vec2,
        response: Dict[str, object],
    ) -> None:
        task.measured_positions.append(position)
        previous_status = task.status
        previous_diameter = task.diameter_m
        previous_polygon = task.polygon
        previous_clear_position = task.clear_position
        result = response.get("measure_result")
        if result == "no_signal":
            task.no_signal_count += 1
            return
        if result == "near":
            task.status = ChannelStatus.LOCATABLE
            task.diameter_m = 0.0
            task.clear_position = position
            task.second_region = None
            return
        if result != "direction":
            raise RuntimeError(f"未知 measure_result: {result}")

        obs = BearingObservation(
            position=position,
            direction_deg=float(response["svd_deg"]),
            virtual_time_s=float(response.get("virtual_time_s", self.state.virtual_time_s)),
        )
        task.observations.append(obs)
        if len(task.observations) == 1:
            task.second_region = self.region_provider.for_first_observation(obs)
            task.status = ChannelStatus.UNLOCATED
            return

        localization = GeometryAdapter.localize(task.observations)
        if localization is None:
            # 新观测发生数值退化时保留此前更好的可定位结果。
            task.status = previous_status if previous_status == ChannelStatus.LOCATABLE else ChannelStatus.UNLOCATED
            return
        if (
            previous_status == ChannelStatus.LOCATABLE
            and previous_diameter is not None
            and localization.diameter_m > previous_diameter
        ):
            task.status = previous_status
            task.diameter_m = previous_diameter
            task.polygon = previous_polygon
            task.clear_position = previous_clear_position
            return
        task.polygon = localization.polygon
        task.diameter_m = localization.diameter_m
        task.clear_position = localization.diameter_midpoint
        task.status = (
            ChannelStatus.LOCATABLE
            if 0.0 < localization.diameter_m <= self.cfg.diameter_limit_m + 1e-8
            else ChannelStatus.UNLOCATED
        )
        if task.status == ChannelStatus.LOCATABLE:
            task.second_region = None

    def _execute_measure_action(self, action: Action) -> None:
        start = self.state.position
        first = True
        for channel in action.channels:
            # 防止旧批次继续测量已经达到 40 m 或已经清除的频道。
            if self.state.channels[channel].status in (
                ChannelStatus.LOCATABLE, ChannelStatus.CLEARED
            ):
                continue
            response = self.client.measure(action.position, channel)
            if first:
                self.state.total_move_distance_m += start.distance_to(action.position)
                self.state.position = action.position
                self.state.trajectory.append(action.position)
                first = False
            self.state.receiver_channel = channel
            self.state.virtual_time_s = float(response.get("virtual_time_s", self.state.virtual_time_s))
            self.state.total_measurements += 1
            self._update_from_measurement(self.state.channels[channel], action.position, response)
            self.state.monitoring_sequence.append({
                "position": action.position.to_dict(),
                "channel": channel,
                "result": response.get("measure_result"),
                "svd_deg": response.get("svd_deg"),
            })
        if action.kind == ActionType.CENTRAL_SCAN:
            self.state.central_scan_done = True
        elif action.kind == ActionType.OUTER_SCAN:
            self.state.outer_waypoint_index += 1

    def _execute_clear_action(self, action: Action) -> None:
        channel = action.channels[0]
        task = self.state.channels[channel]
        start = self.state.position
        response = self.client.clear(action.position, channel)
        self.state.total_move_distance_m += start.distance_to(action.position)
        self.state.position = action.position
        self.state.trajectory.append(action.position)
        self.state.virtual_time_s = float(response.get("virtual_time_s", self.state.virtual_time_s))
        self.state.total_clear_attempts += 1
        if response.get("clear_result") == "success":
            task.status = ChannelStatus.CLEARED
            task.second_region = None
            self.state.successful_clears += 1
            self.state.clear_order.append(channel)
        else:
            # 直径<=40并不必然保证直径圆覆盖四边形；失败后继续补充测向。
            task.clear_failures += 1
            task.status = ChannelStatus.UNLOCATED
            task.measured_positions.append(action.position)

    def execute(self, action: Action) -> None:
        start = self.state.position
        if action.kind in (
            ActionType.CENTRAL_SCAN,
            ActionType.OUTER_SCAN,
            ActionType.MEASURE_BATCH,
        ):
            self._execute_measure_action(action)
        else:
            self._execute_clear_action(action)

        self.state.radial_frontier_m = max(
            self.state.radial_frontier_m,
            self._radius(self.state.position),
        )

        # 记录和打印在 run() 中完成；此处保留 start 供调试。
        _ = start

    # ---------- 日志与终止 ----------

    def _detected_uncleared_count(self) -> int:
        return sum(
            bool(t.observations) and t.status != ChannelStatus.CLEARED
            for t in self.state.channels.values()
        )

    def _locatable_count(self) -> int:
        return sum(t.status == ChannelStatus.LOCATABLE for t in self.state.channels.values())

    @staticmethod
    def _action_dict(action: Action) -> Dict[str, object]:
        return {
            "kind": action.kind.value,
            "position": action.position.to_dict(),
            "channels": list(action.channels),
            "utility": action.utility,
            "estimated_time_s": action.estimated_time_s,
            "move_distance_m": action.move_distance_m,
            "information_count": action.information_count,
            "route_detour_m": action.route_detour_m,
            "radial_backtrack_m": action.radial_backtrack_m,
            "details": action.details,
        }

    def _write_jsonl(self, obj: Dict[str, object]) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(obj, ensure_ascii=False) + "\n")

    def _record_step(
        self,
        step: int,
        start: Vec2,
        action: Action,
        candidates: Sequence[Action],
    ) -> None:
        record = StepRecord(
            step=step,
            action=self._action_dict(action),
            start_position=start.to_dict(),
            end_position=self.state.position.to_dict(),
            uncleared_detected_channels=self._detected_uncleared_count(),
            locatable_channels=self._locatable_count(),
            candidate_actions=[self._action_dict(a) for a in candidates[:20]],
            move_distance_m=start.distance_to(self.state.position),
            estimated_step_time_s=action.estimated_time_s,
            actual_virtual_time_s=self.state.virtual_time_s,
            cumulative_move_distance_m=self.state.total_move_distance_m,
            cumulative_measurements=self.state.total_measurements,
            cumulative_clears=self.state.successful_clears,
        )
        self.state.records.append(record)
        self._write_jsonl(asdict(record))

        print("\n" + "=" * 78)
        print(f"步骤 {step}  当前位置=({start.x:.1f},{start.y:.1f})")
        print(
            f"当前未清除已发现频道={record.uncleared_detected_channels}，"
            f"可清除频道={record.locatable_channels}"
        )
        print("候选动作（按评分排序）：")
        for idx, candidate in enumerate(candidates[:self.cfg.print_top_candidates], 1):
            print(f"  {idx:2d}. {candidate.short()}")
        print("最终选择：", action.short())
        print(
            f"本步移动距离={record.move_distance_m:.2f} m，"
            f"预计本步耗时={record.estimated_step_time_s:.2f} s，"
            f"累计虚拟时间={record.actual_virtual_time_s:.2f} s"
        )

    def _mission_complete(self) -> bool:
        discovery_done = (
            self.state.central_scan_done
            and self.state.outer_waypoint_index >= len(self.outer_waypoints)
        )
        active = any(
            t.status in (ChannelStatus.UNLOCATED, ChannelStatus.LOCATABLE)
            for t in self.state.channels.values()
        )
        # 题面给出干扰源总数为 10--16；少于 10 个成功清除时不能仅凭 no_signal 宣告完成。
        enough_sources_cleared = self.state.successful_clears >= self.cfg.expected_source_min
        return discovery_done and not active and enough_sources_cleared

    def run(self) -> Dict[str, object]:
        if self.log_path.exists():
            self.log_path.unlink()
        enter_response = self.client.enter()
        self.state.virtual_time_s = float(enter_response.get("virtual_time_s", 0.0))
        print("进入成功，现实可用时间：", enter_response.get("remaining_real_duration_s"), "秒")

        stopped_reason = "max_steps"
        try:
            for step in range(1, self.cfg.max_planning_steps + 1):
                if self._mission_complete():
                    stopped_reason = "all_detected_targets_cleared_after_outward_sweep"
                    break
                candidates = self.generate_actions()
                if not candidates:
                    stopped_reason = "no_feasible_action"
                    break
                action = candidates[0]
                start = self.state.position
                self.execute(action)
                self._record_step(step, start, action, candidates)
        finally:
            try:
                exit_response = self.client.exit()
                exit_reason = exit_response.get("exit_reason", "unknown")
            except Exception as exc:
                exit_reason = f"exit_failed: {exc}"

        summary = self.summary(stopped_reason, str(exit_reason))
        summary_path = self.log_path.with_name(self.log_path.stem + "_summary.json")
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print("\n" + "=" * 78)
        print("任务结束：", stopped_reason)
        print(f"成功清除顺序：{self.state.clear_order}")
        print(f"总移动距离：{self.state.total_move_distance_m:.2f} m")
        print(f"总检测次数：{self.state.total_measurements}")
        print(f"总清除尝试：{self.state.total_clear_attempts}")
        print(f"总任务时间：{self.state.virtual_time_s:.2f} s")
        print("详细日志：", self.log_path)
        print("汇总文件：", summary_path)
        return summary

    def summary(self, stopped_reason: str, exit_reason: str) -> Dict[str, object]:
        return {
            "stopped_reason": stopped_reason,
            "exit_reason": exit_reason,
            "final_position": self.state.position.to_dict(),
            "trajectory": [p.to_dict() for p in self.state.trajectory],
            "monitoring_sequence": self.state.monitoring_sequence,
            "clear_order": self.state.clear_order,
            "total_move_distance_m": self.state.total_move_distance_m,
            "total_measurements": self.state.total_measurements,
            "total_clear_attempts": self.state.total_clear_attempts,
            "successful_clears": self.state.successful_clears,
            "total_task_time_s": self.state.virtual_time_s,
            "central_scan_done": self.state.central_scan_done,
            "outer_waypoint_index": self.state.outer_waypoint_index,
            "outer_waypoints": [p.to_dict() for p in self.outer_waypoints],
            "radial_frontier_m": self.state.radial_frontier_m,
            "channel_states": {
                str(ch): {
                    "status": task.status.value,
                    "observations": [
                        {
                            "position": o.position.to_dict(),
                            "direction_deg": o.direction_deg,
                            "virtual_time_s": o.virtual_time_s,
                        }
                        for o in task.observations
                    ],
                    "diameter_m": task.diameter_m,
                    "clear_position": task.clear_position.to_dict()
                    if task.clear_position else None,
                    "no_signal_count": task.no_signal_count,
                    "clear_failures": task.clear_failures,
                }
                for ch, task in self.state.channels.items()
            },
        }


# ============================================================================
# 命令行入口
# ============================================================================


def _mock_sources() -> List[MockSource]:
    return [
        MockSource(2, Vec2(420.0, 720.0), 1450.0),
        MockSource(4, Vec2(-680.0, 580.0), 1320.0),
        MockSource(7, Vec2(820.0, -360.0), 1500.0),
        MockSource(9, Vec2(-260.0, -920.0), 1180.0),
        MockSource(13, Vec2(80.0, 1280.0), 1400.0),
        MockSource(14, Vec2(1180.0, 420.0), 1500.0),
        MockSource(15, Vec2(-1220.0, 180.0), 1380.0),
        MockSource(16, Vec2(610.0, -1120.0), 1250.0),
        MockSource(18, Vec2(-880.0, -760.0), 1480.0),
        MockSource(20, Vec2(250.0, 260.0), 1100.0),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="第三问动态搜索与清除策略")
    parser.add_argument("--mode", choices=("mock", "http"), default="mock")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--robot-id", default=os.environ.get("ROBOT_ID", ""))
    parser.add_argument("--log", default="t3_mission_log.jsonl")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = PlannerConfig()

    # T2 区域预处理必须放在 /enter 前，避免占用正式测试的现实计时窗口。
    print("正在加载/生成第二问标准候选区域……")
    region_provider = SecondRegionProvider(cfg)

    if args.mode == "http":
        client: RobotClient = HttpRobotClient(args.base_url, args.robot_id)
    else:
        client = MockRobotClient(_mock_sources(), cfg)

    planner = RollingPlanner(client, cfg, region_provider, Path(args.log))
    planner.run()


if __name__ == "__main__":
    main()
