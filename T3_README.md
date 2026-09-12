# 第三问动态搜索与清除策略

## 1 程序架构

程序分为四层：

1. 前两问适配层
   - `GeometryAdapter.localize()`：调用 `T1.compute_feasible_vertices()`、`T1.convex_hull()` 和 `T1.polygon_diameter()`。
   - `SecondRegionProvider`：优先读取 `T2.py` 生成的 `t2_results.json`；不存在时，在调用 `/enter` 前调用 `T2` 的目标枚举、候选评分和分类函数生成标准候选区域。
   - 第一问的几何算法和第二问的评分算法均未在第三问中重新实现。

2. 任务状态层
   - `ChannelTask` 保存每个频道的状态、历史测向、候选区域、定位多边形、直径和清除位置。
   - `MissionState` 保存机器狗位置、当前测向机频道、顺时针扇区游标、轨迹、累计时间和动作记录。

3. 滚动决策层
   - `INITIAL_SCAN`：前往预设初始探测点，对尚需探测的频道连续扫描。
   - `MEASURE_BATCH`：移动一次后，在同一位置为多个频道依次完成二次测向。
   - `CLEAR`：前往已满足定位精度的目标位置进行光学搜索和清除。
   - 三种动作都使用统一的 `Action` 数据结构，并在每一步重新评分。

4. 执行接口层
   - `HttpRobotClient`：连接正式模拟器的 `/enter`、`/measure`、`/clear` 和 `/exit`。
   - `MockRobotClient`：离线验证调度逻辑，不访问模拟器。

## 2 核心数据结构

每个频道的状态为：

- `UNKNOWN`：尚未得到有效示向度；`no_signal` 不会直接判定该频道无干扰源。
- `UNLOCATED`：已有有效示向度，但测向次数不足或定位直径大于 40 m。
- `LOCATABLE`：定位区域直径不大于 40 m，可以执行光学搜索和清除。
- `CLEARED`：模拟器返回 `clear_result="success"`，该频道从后续测向评分中删除。

联合二次测向动作 `MEASURE_BATCH(P, channels)` 表示机器狗只移动一次到达位置 `P`，然后在原地依次调用多个频道的 `/measure`。评分中的检测时间和换频道时间均按实际调用次数计算。

## 3 清晰伪代码

```text
输入：
    第一问几何模块 T1
    第二问候选区域模块 T2
    初始探测点集合 Q
    频道集合 C = {1,2,...,20}
    集中参数 Config

预处理：
    在调用 /enter 之前加载或生成 T2 的标准第二监测区域
    将区域按第一次测向位置和方向进行平移、旋转后复用

初始化：
    对每个频道 i：state[i] <- UNKNOWN
    robot_position <- (0,0)
    receiver_channel <- 1
    sector_cursor <- 起始扇区
    trajectory <- [(0,0)]
    调用 /enter

while 未满足终止条件 and step < MAX_STEPS：

    # A. 生成初始扫描动作
    initial_actions <- empty
    for 每个尚未访问的初始探测点 q：
        channels <- 所有 UNKNOWN 或 UNLOCATED 频道
        time <- move_time(robot_position,q)
                + measure_time * len(channels)
                + channel_switch_time
        utility <- discovery_weight * unknown_count / time
                   - sector_penalty(q)
        加入 INITIAL_SCAN(q, channels)

    # B. 生成联合二次测向动作
    regions <- 所有 UNLOCATED 频道的第二监测区域
    candidate_points <- 合并各区域的代表采样点

    for 每个候选位置 P：
        served <- {i | P 属于 R_i 且频道 i 未在 P 附近测量过}

        if 频道 i 已经二次测向但直径仍大于 40 m：
            在当前定位中心周围产生局部环形复测点
            用局部复测点替代沿原第二监测区域继续试探

        n(P) <- len(served)
        time <- move_time(robot_position,P)
                + batch_measure_time(served)
        utility <- information_weight * n(P) / time
                   + overlap_bonus * max(0,n(P)-1)
                   - sector_penalty(P)
        加入 MEASURE_BATCH(P, served)

    best_measure <- 测向动作中 utility 最大者

    # C. 生成清除动作
    clear_actions <- empty
    for 每个 LOCATABLE 频道 i：
        P_i <- 定位区域直径端点的中点
        time <- move_time(robot_position,P_i)
                + optical_time + clear_time

        if P_i 几乎位于前往 best_measure 的路径上：
            on_route_bonus <- 正值
        else：
            on_route_bonus <- 0

        utility <- clear_weight / time
                   + on_route_bonus
                   - sector_penalty(P_i)
        加入 CLEAR(P_i,i)

    # D. 前中后期权重自动切换
    根据已发现、未定位、可清除和已清除频道比例：
        前期提高 information_weight
        后期提高 clear_weight

    # E. 统一选择与执行
    actions <- initial_actions + measure_actions + clear_actions
    action_star <- argmax utility(action)

    if action_star 是测向动作：
        移动到目标位置
        按优化后的频道顺序逐个调用 /measure
        对 direction：保存示向度
        对 near：将当前位置作为清除位置，状态改为 LOCATABLE
        对 no_signal：只增加无信号次数，不判定频道不存在

        if 某频道第一次获得 direction：
            调用 T2 生成刚体变换后的第二监测区域

        if 某频道至少获得两次 direction：
            调用 T1 计算定位多边形及直径 D_i
            if 0 < D_i <= 40 m：
                state[i] <- LOCATABLE
            else：
                state[i] <- UNLOCATED

    else if action_star 是清除动作：
        移动到清除位置并调用 /clear
        if clear_result == success：
            state[i] <- CLEARED
            立即从后续测向评分中删除频道 i
        else：
            state[i] <- UNLOCATED
            生成补充测向任务

    更新：
        当前机器狗位置
        当前测向机频道
        顺时针扇区游标
        轨迹、监测点序列和清除顺序
        移动距离、检测次数、清除次数和累计时间

    输出本步状态、候选动作评分和最终动作
    回到循环开头，重新规划

终止：
    已完成全部初始覆盖扫描
    已成功清除数量不少于题面给出的最小干扰源数量 10
    不再存在 UNLOCATED 或 LOCATABLE 频道

调用 /exit
输出完整轨迹、监测序列、清除顺序、总距离、总检测次数和总时间
```

## 4 评分函数

联合测向评分采用：

```text
J_measure(P)
    = w_info * n(P) / T_batch(P)
      + w_overlap * max(0,n(P)-1)
      - sector_penalty(P)
```

其中 `T_batch(P)` 包含一次移动、所有频道的检测耗时和实际换频道耗时。

清除评分采用：

```text
J_clear(i)
    = w_clear / T_clear(i)
      + on_route_bonus(i)
      - sector_penalty(i)
```

所有权重均集中在 `PlannerConfig` 中，可直接进行单因素或多因素敏感性分析。

## 5 运行方法

将以下三个文件放在同一目录：

```text
T1.py
T2.py
T3_dynamic_search.py
```

先进行离线测试：

```bash
python T3_dynamic_search.py --mode mock
```

连接正式模拟器前，在模拟器中启动测试并等待接口就绪，然后运行：

```bash
python T3_dynamic_search.py \
  --mode http \
  --base-url http://127.0.0.1:2026 \
  --robot-id 你的参赛队号
```

也可以通过环境变量提供参赛队号：

```bash
set ROBOT_ID=你的参赛队号
python T3_dynamic_search.py --mode http
```

Windows PowerShell 对应写法为：

```powershell
$env:ROBOT_ID="你的参赛队号"
python T3_dynamic_search.py --mode http
```

## 6 输出文件

默认生成：

- `t3_mission_log.jsonl`：每一步的位置、候选动作、评分、最终动作、移动距离和累计时间；
- `t3_mission_log_summary.json`：完整轨迹、监测序列、清除顺序、总移动距离、总检测次数、总清除次数和总任务时间。

可通过 `--log` 指定日志路径：

```bash
python T3_dynamic_search.py --mode mock --log result/run1.jsonl
```

## 7 使用前注意

1. `SecondRegionProvider` 的预处理位于 `/enter` 之前，不占用正式测试的 20 分钟现实运行时间。
2. HTTP 请求严格串行发送；网络异常重试时复用同一请求体和 `request_id`。
3. `no_signal` 不能说明频道无干扰源，因此程序不会据此把频道标记为已清除或不存在。
4. 清除失败后，频道会退回 `UNLOCATED`，重新参与补充测向评分。
5. 当前第一版采用直径端点中点作为光学搜索中心；若后续加入最小包围圆模块，可直接替换 `clear_position` 的生成方式，不需要改动调度器。
6. 正式测试前应先使用演练模式检查参数、日志大小和终止条件，不要直接消耗正式测试机会。
