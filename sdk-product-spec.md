# Linkerbot 全机型 SDK 产品实现文档（协议 + Manager + 指令码）

## 1. 文档目标

本文档用于指导开发者从零实现 Linkerbot 全系列灵巧手 SDK（L6 / L20lite / L25 / O6 / O7）。

目标是保证实现结果与当前 Python SDK 行为一致，尤其是：

- manager 功能边界一致
- 指令码与载荷格式一致
- 三种读取模式（阻塞 / 流式 / 缓存）一致
- 不同机型的能力差异被明确处理（避免“某机型不支持但被误实现”）

---

## 2. 总体架构与通信约束

### 2.1 设备入口对象

每个设备入口对象（`L6` / `L20lite` / `L25` / `O6` / `O7`）具备：

- manager 聚合（`angle`、`speed`、`torque`、`temperature` 等）
- 统一快照：`get_snapshot()`
- 统一流：`stream()` / `stop_stream()`
- 后台轮询：`start_polling()` / `stop_polling()`
- 生命周期：`close()` / `is_closed()`，支持 context manager

### 2.2 CAN 约束

- 传输层：`python-can`，标准帧（`is_extended_id=False`）
- 设备 ID：
  - 右手：`0x27`
  - 左手：`0x28`
- 发送模型：带发送队列的分发器，接收与发送线程分离
- SDK 内部对同 arbitration_id + 指令码进行过滤与解析

### 2.3 数据中继模型（DataRelay）

每个“可读 manager”统一包含：

- 最新值缓存（snapshot）
- 阻塞等待下一帧（wait）
- 事件 sink（用于统一 stream）

---

## 3. 三种读取模式（统一定义）

### 3.1 阻塞读取（Blocking）

- API：`manager.get_blocking(timeout_ms=...)`
- 行为：发送查询指令，阻塞直到收到完整响应或超时
- 超时：抛出 `TimeoutError`

### 3.2 缓存读取（Cached）

- API：`manager.get_snapshot()` 或 `hand.get_snapshot()`
- 行为：不发请求，直接返回最近一次缓存
- 初始状态：未收到数据前返回 `None`

### 3.3 流式读取（Streaming）

- API：`hand.stream()`（返回 `IterableQueue`）
- 数据来源：仅来自轮询线程（`start_polling()`）触发的 manager 响应
- 队列满时策略：丢弃最旧事件，再尝试写入新事件

注意：manager 本身没有公共 `stream()`；流式读取属于 hand 级统一事件流。

---

## 4. Manager API 清单（按机型）

## 4.1 L6

- `angle`：`set_angles` / `get_blocking` / `get_snapshot`
- `speed`：`set_speeds`（仅设置，不支持读）
- `torque`：`set_torques` / `get_blocking` / `get_snapshot`
- `temperature`：`get_blocking` / `get_snapshot`
- `current`：`get_blocking` / `get_snapshot`
- `fault`：`clear_faults` / `get_blocking` / `get_snapshot`
- `force_sensor`：`get_blocking` / `get_snapshot` / `get_finger`
- `version`：`get_device_info`

## 4.2 L20lite

- `angle`：`set_angles` / `get_blocking` / `get_snapshot`
- `speed`：`set_speeds` / `get_blocking` / `get_snapshot`
- `torque`：`set_torques` / `get_blocking` / `get_snapshot`
- `temperature`：`get_blocking` / `get_snapshot`
- `fault`：`get_blocking` / `get_snapshot`（无清故障）
- `force_sensor`：`get_blocking` / `get_snapshot` / `get_finger`
- `version`：`get_device_info`

## 4.3 L25

- `angle`：`set_angles` / `get_blocking` / `get_snapshot`
- `speed`：`set_speeds` / `get_blocking` / `get_snapshot`
- `torque`：`set_torques` / `get_blocking` / `get_snapshot`
- `temperature`：`get_blocking` / `get_snapshot`
- `fault`：`clear_faults` / `get_blocking` / `get_snapshot`
- `force_sensor`：`get_blocking` / `get_snapshot` / `get_finger`
- `version`：`get_device_info`

## 4.4 O6

- `angle`：`set_angles` / `get_blocking` / `get_snapshot`
- `speed`：`set_speeds` / `get_blocking` / `get_snapshot`
- `acceleration`：`set_accelerations` / `get_blocking` / `get_snapshot`
- `torque`：`set_torques` / `get_blocking` / `get_snapshot`
- `temperature`：`get_blocking` / `get_snapshot`
- `fault`：`get_blocking` / `get_snapshot`
- `force_sensor`：`get_blocking` / `get_snapshot` / `get_finger`
- `version`：`get_device_info`

## 4.5 O7

- `angle`：`set_angles` / `get_blocking` / `get_snapshot`
- `speed`：`set_speeds` / `get_blocking` / `get_snapshot`
- `acceleration`：`set_accelerations` / `get_blocking` / `get_snapshot`
- `torque`：`set_torques` / `get_blocking` / `get_snapshot`
- `temperature`：`get_blocking` / `get_snapshot`
- `fault`：`get_blocking` / `get_snapshot`
- `force_sensor`：`get_blocking` / `get_snapshot` / `get_finger`
- `version`：`get_device_info`

---

## 5. 统一流（stream）能力矩阵

| 机型 | 可轮询/流式的 `SensorSource` |
|---|---|
| L6 | `ANGLE` / `TORQUE` / `TEMPERATURE` / `CURRENT` / `FAULT` / `FORCE_SENSOR` |
| L20lite | `ANGLE` / `SPEED` / `TORQUE` / `TEMPERATURE` / `FORCE_SENSOR` |
| L25 | `ANGLE` / `SPEED` / `TORQUE` / `TEMPERATURE` / `FAULT` / `FORCE_SENSOR` |
| O6 | `ANGLE` / `TORQUE` / `SPEED` / `ACCELERATION` / `TEMPERATURE` / `FAULT` / `FORCE_SENSOR` |
| O7 | `ANGLE` / `TORQUE` / `SPEED` / `ACCELERATION` / `TEMPERATURE` / `FAULT` / `FORCE_SENSOR` |

关键差异：

- L20lite 虽有 `fault manager`，但不在统一流/统一快照中
- L6 的 `speed` 仅写入控制，不支持读取
- `version` 在所有机型都不进入 stream/snapshot，仅阻塞查询

---

## 6. 指令码规范（按机型）

说明：以下“操作”列中，`设`=设置参数，`读`=读取状态。

## 6.1 L6

| 指令码 | manager | 用途 | 请求格式 | 响应要点 | 操作 |
|---|---|---|---|---|---|
| `0x01` | angle | 角度控制/读取 | `[0x01, a1..a6]` 或 `[0x01]` | `[0x01, v1..v6]` | 设+读 |
| `0x02` | torque | 扭矩控制/读取 | `[0x02, t1..t6]` 或 `[0x02]` | `[0x02, v1..v6]` | 设+读 |
| `0x05` | speed | 速度控制 | `[0x05, s1..s6]` | 无专用读路径 | 设 |
| `0x33` | temperature | 温度读取 | `[0x33]` | `[0x33, v1..v6]` | 读 |
| `0x36` | current | 电流读取 | `[0x36]` | `[0x36, v1..v6]` | 读 |
| `0x35` | fault | 故障读取 | `[0x35]` | `[0x35, v1..v6]` | 读 |
| `0x83` | fault | 清故障 | `[0x83,1,1,1,1,1,1]` | fire-and-forget | 设 |
| `0xB1..0xB5 + 0xC6` | force_sensor | 五指力传感查询 | `[finger_cmd,0xC6]` | 12 帧，每帧 6 byte，shape `(12,6)` | 读 |
| `0xC0` | version | SN 查询 | `[0xC0]` | SN 分 4 帧组装 | 读 |
| `0xC1` | version | PCB 版本 | `[0xC1,0x01]` | 响应含 `0x01` 标记位 | 读 |
| `0xC2` | version | 固件版本 | `[0xC2]` | `major/minor/patch` | 读 |
| `0xC4` | version | 机械版本 | `[0xC4]` | `major/minor/patch` | 读 |

## 6.2 L20lite

| 指令码 | manager | 用途 | 操作 |
|---|---|---|---|
| `0x01` + `0x04` | angle | 两帧覆盖 10 关节 | 设+读 |
| `0x05` + `0x06` | speed | 两帧覆盖 10 关节 | 设+读 |
| `0x02` + `0x03` | torque | 两帧覆盖 10 关节 | 设+读 |
| `0x33` + `0x34` | temperature | 两帧温度读取 | 读 |
| `0x35` + `0x36` | fault | 两帧故障读取（无清故障） | 读 |
| `0xB1..0xB5 + 0xC6` | force_sensor | 五指，shape `(12,6)` | 读 |
| `0xC0` / `0xC1` / `0xC2` / `0xC4` | version | SN/PCB/FW/机械 | 读 |

补充：L20lite 的 SN 响应使用 `byte_index`（`0,6,12,18`）拼帧。

## 6.3 L25

| 指令码 | manager | 用途 | 操作 |
|---|---|---|---|
| `0x41..0x45` | angle | 五帧覆盖 16 DoF | 设+读 |
| `0x49..0x4D` | speed | 五帧覆盖 16 DoF | 设+读 |
| `0x51..0x55` | torque | 五帧覆盖 16 DoF | 设+读 |
| `0x59..0x5D` | fault | 五帧故障读取 | 读 |
| `0x59..0x5D` | fault | 五帧清故障（每帧 `[cmd,1,1,1,1,1,1]`） | 设 |
| `0x61..0x65` | temperature | 五帧温度读取 | 读 |
| `0xB1..0xB5 + 0xC6` | force_sensor | 五指，shape `(12,6)` | 读 |
| `0xC0` / `0xC1` / `0xC2` / `0xC4` | version | SN/PCB/FW/机械 | 读 |

## 6.4 O6

| 指令码 | manager | 用途 | 操作 |
|---|---|---|---|
| `0x01` | angle | 角度控制/读取（6 关节） | 设+读 |
| `0x02` | torque | 扭矩控制/读取（6 关节） | 设+读 |
| `0x05` | speed | 速度控制/读取（6 关节） | 设+读 |
| `0x87` | acceleration | 加速度控制/读取（6 关节） | 设+读 |
| `0x33` | temperature | 温度读取（6 关节） | 读 |
| `0x35` | fault | 故障读取（6 关节） | 读 |
| `0xB1..0xB5 + 0xA4` | force_sensor | 五指，shape `(10,4)` | 读 |
| `0xC0` / `0xC1` / `0xC2` / `0xC4` | version | SN/PCB/FW/机械 | 读 |

补充：O6 的 `0xC1` 请求仅发 `[0xC1]`（无 L6 的 `0x01` 子参数）。

## 6.5 O7

| 指令码 | manager | 用途 | 操作 |
|---|---|---|---|
| `0x01` | angle | 角度控制/读取（7 关节） | 设+读 |
| `0x02` | torque | 扭矩控制/读取（7 关节） | 设+读 |
| `0x05` | speed | 速度控制/读取（7 关节） | 设+读 |
| `0x87` | acceleration | 加速度控制/读取（7 关节） | 设+读 |
| `0x33` | temperature | 温度读取（7 关节） | 读 |
| `0x35` | fault | 故障读取（7 关节） | 读 |
| `0xB1..0xB5 + 0xC6` | force_sensor | 五指，shape `(12,6)` | 读 |
| `0xC0` / `0xC1` / `0xC2` / `0xC4` | version | SN/PCB/FW/机械 | 读 |

---

## 7. 关键数据转换与解析规则

## 7.1 角度/速度/扭矩（常规）

- 常规归一化范围：`0..100`
- 常规编码：`raw = round(v * 255 / 100)`
- 常规解码：`v = raw * 100 / 255`

适用：L6/L20lite/L25 的 angle/speed/torque，以及 O6/O7 的 angle/speed/torque。

## 7.2 O6/O7 速度与扭矩的物理单位辅助

- 速度：`1 raw = 0.732 RPM`，最大约 `186.66 RPM`
- 扭矩电流限：`1 raw = 6.5 mA`，最大约 `1657.5 mA`

## 7.3 O6/O7 加速度特殊映射

- 指令：`0x87`
- 特殊点：硬件 `0` 代表最大加速度
- 映射：
  - 用户 `100` -> 硬件 `0`
  - 用户 `0..99` -> 硬件 `1..254`（线性）

## 7.4 温度与电流

- 温度：按 byte 直接作为摄氏温度值
- L6 电流：`mA = raw * 1400 / 255`

## 7.5 故障码位定义

- L6：`0x7F` 有效位（BIT0..BIT6）
- O6/O7/L20lite：`0x2F` 有效位（BIT0..BIT3, BIT5）
- L25：使用完整 byte（BIT0..BIT7）+ 独立语义命名

## 7.6 力传感帧组装

- 指令前缀：`0xB1..0xB5`（拇/食/中/无名/小）
- 帧索引：`msg.data[1] >> 4`
- O6：10 帧 * 4 byte -> `(10,4)`
- 其他机型：12 帧 * 6 byte -> `(12,6)`
- 仅当整组帧齐全时推送一次数据

## 7.7 Version 读取顺序

`get_device_info()` 的查询顺序固定为：

1. firmware
2. mechanical
3. pcb
4. serial number

应保持顺序发送，避免设备端被短时突发请求压垮。

---

## 8. 实现要求（必须满足）

- 所有 `timeout_ms` 必须校验 `> 0`，否则抛 `ValidationError`
- 所有 set 类 API 必须校验输入长度与数值范围
- manager 回调必须先过滤 arbitration_id，再过滤指令码和长度
- 多帧 manager（L20lite/L25 全部多帧项）需做“帧齐再出数”
- 多帧 manager 需实现 in-flight 节流（当前实现窗口约 `0.2s`）
- `start_polling()` 再次调用时必须先停掉上一轮
- `stop_polling()` / `stop_stream()` 必须幂等
- hand close 后调用核心 API 应抛 `StateError`

---

## 9. SDK 交付验收清单

满足以下条目可视为“可完成全机型 SDK”：

- 指令码与本文档一致（含多帧分片）
- manager API 清单与行为一致
- 三种读模式行为一致（含 stream 丢弃最旧策略）
- 五机型能力差异正确实现（尤其 L6 speed 只写、L20lite fault 不入 stream）
- 力传感矩阵维度与拼帧规则正确
- version 查询差异（L6 的 `0xC1,0x01`）正确
- 异常模型与超时行为一致

---

## 10. 当前项目中的已知差异与测试覆盖

- 仓库测试目录包含：`L6`、`L20lite`、`L25`、`O6`
- `O7` 当前无同级硬件测试目录，建议在实现新 SDK 时补齐 O7 全套回归

