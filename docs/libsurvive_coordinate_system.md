# libsurvive 坐标系统详解

本文档详细说明 libsurvive 标定后的坐标系定义，以及 Vive Tracker 本体坐标系与世界坐标系的关系。

## 1. 标定原理概述

Vive Tracker 使用 **Lighthouse 激光定位系统**：
- 两个 Lighthouse 基站发射红外激光扫描
- Tracker 上的光敏传感器检测激光
- 通过激光到达的时间差计算 Tracker 在空间中的位置和姿态

**标定的目的**：确定两个 Lighthouse 之间的相对位置关系，从而建立统一的世界坐标系。

## 2. 标定算法流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        driver_vive.c                                │
│                    (USB 驱动读取原始数据)                            │
└─────────────────────────────────────────────────────────────────────┘
                                ↓
┌─────────────────────────────────────────────────────────────────────┐
│              driver_global_scene_solver.c                           │
│                    (全局场景求解器)                                  │
│  - 收集多帧 Tracker 静止时的光信号数据 (scenes)                     │
│  - 当数据足够时触发优化求解                                          │
└─────────────────────────────────────────────────────────────────────┘
                                ↓
┌─────────────────────────────────────────────────────────────────────┐
│                     poser_mpfit.c                                   │
│              (非线性最小二乘优化 - MPFit)                            │
│  - 同时优化所有 Lighthouse 位姿和 Tracker 位姿                      │
│  - 最小化激光角度测量误差 (Levenberg-Marquardt)                     │
└─────────────────────────────────────────────────────────────────────┘
                                ↓
┌─────────────────────────────────────────────────────────────────────┐
│                    配置保存                                          │
│              ~/.config/libsurvive/config.json                       │
└─────────────────────────────────────────────────────────────────────┘
```

## 3. 世界坐标系定义

### 3.1 默认模式 (`--origin tracker`)

libsurvive 源码 `poser.c` 中的注释：

```c
// Assume that the space solved for is valid but completely arbitrary. We are going to do a few things:
// a) Using the gyro data, normalize it so that gravity is pushing straight down along Z
// c) Assume the object is at origin
// b) Place the first lighthouse on the X axis by rotating around Z
```

**坐标系定义：**

```
        Z (向上, 与重力相反)
        │
        │
        │      Y (LH0 方向)
        │     /
        │    /
        │   /
        └──────────── X
       原点 (Tracker 标定时的位置)
```

| 轴 | 定义 | 确定方式 |
|---|------|----------|
| **原点** | Tracker 标定时所在位置 | 由 `--force-calibrate` 时 Tracker 位置决定 |
| **+Z** | 向上（与重力相反）| 由 IMU 加速度计测量的重力方向决定 |
| **+Y** | 指向 Lighthouse 0 (LH0) | 旋转坐标系使 LH0 在 +Y 轴上（90度） |
| **+X** | 右手坐标系 | 由 Z × Y 叉乘确定 |

**关键代码 (`poser.c`)：**

```c
// 用加速度计确定 "up" 方向 (第 178-179 行)
quatfrom2vectors(object2objUp.Rot, so->activations.accel, up);

// 旋转使 LH0 在 Y 轴上 (第 190-192 行)
FLT ang = atan2(lighthouse2objUp.Pos[1], lighthouse2objUp.Pos[0]);
FLT ang_target = M_PI / 2.;  // 90度 = Y轴方向
FLT euler[3] = {0, 0, ang_target - ang};
```

### 3.2 Lighthouse 原点模式 (`--center-on-lh0`)

```c
if (centerOnLh0) {
    // 原点移到 LH0
    sub3d(obj2world.Pos, obj2world.Pos, lighthouse2world.Pos);
    lighthouse2world.Pos[0] = lighthouse2world.Pos[1] = lighthouse2world.Pos[2] = 0.0;

    // +X 指向 LH0 的朝向 (相机光轴方向)
    LinmathPoint3d camFwd = {0, 0, -1}, worldFwd = {0};
    ApplyPoseToPoint(worldFwd, &lighthouse2world, camFwd);
}
```

**坐标系定义：**

```
        Z (向上)
        │
        │
        │          
        │         
        │        
        └──────────── X (LH0 朝向)
       LH0 (原点)
```

| 轴 | 定义 |
|---|------|
| **原点** | Lighthouse 0 (LH0) 位置 |
| **+Z** | 向上（与重力相反）|
| **+X** | LH0 的朝向（相机光轴方向）|
| **+Y** | 右手坐标系 |

### 3.3 两种模式对比

| 特性 | `--origin tracker` | `--origin lh0` |
|------|-------------------|----------------|
| 原点 | Tracker 标定位置 | LH0 位置 |
| 适用场景 | 机器人随动应用 | 固定房间坐标系 |
| 重现性 | 同一水平面标定结果一致 | LH0 位置固定则结果一致 |
| 命令 | `python calibrate_vive.py` | `python calibrate_vive.py --origin lh0` |

## 4. Vive Tracker 本体坐标系

根据 HTC 官方文档，Vive Tracker 本体坐标系定义如下：

```
                    +Z (向上, 垂直于安装面)
                     │
                     │
                     │    +Y (向后, 远离 Pogo Pin)
                     │   /
                     │  /
                     │ /
    ┌────────────────┼/────────────────┐
    │                B ───────────────→ +X (指向 Pogo Pin 方向)
    │            (原点)                │
    │         1/4" 螺丝孔               │
    │                                  │
    │    ┌──────────────────────┐     │  ← Pogo Pin 连接器
    │    │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │     │
    │    └──────────────────────┘     │
    └──────────────────────────────────┘
```

### 坐标系参考点

- **基准面 A**：顶部的圆环面，在 1/4 英寸螺母附近
- **基准点 B（原点）**：标准相机安放点（1/4 英寸螺丝）中心线和 A 的交点
- **基准点 C**：稳定针槽的中心线与 A 的交点

| 轴 | 方向 | 参考特征 |
|---|------|----------|
| **原点 B** | 1/4 英寸相机螺丝孔中心 | 标准相机安放点 |
| **+X** | B → C 方向 | 指向 Pogo Pin 连接器 |
| **+Y** | 向后 | 远离 Pogo Pin 连接器 |
| **+Z** | 向上 | 垂直于安装基准面 A |

## 5. 位姿变换关系

### 5.1 从 libsurvive 获取的位姿

libsurvive 输出的位姿 `[position, rotation]` 表示：

```
World_point = R(rotation) × Tracker_body_point + position
```

- **position**: Tracker 原点 B 在世界坐标系中的位置 `[x, y, z]`
- **rotation**: 将 Tracker 本体坐标系旋转到世界坐标系的四元数 `[qx, qy, qz, qw]`

### 5.2 Vive Tracker 到 End-Effector 变换

如果 Vive Tracker 安装在机械臂末端，需要一个固定的变换矩阵 `vive2ee`：

```
T_ee = T_vive @ T_vive2ee
```

示例标定值：
```python
# [x, y, z, qw, qx, qy, qz]
vive2ee = [0.000, 0.021, 0.160, 0.676, -0.207, 0.207, 0.676]
```

### 5.3 与机器人坐标系对齐

要将 Vive 坐标系与机器人基坐标系对齐，使用初始位姿对齐：

```python
# 第一帧时记录
vive_init_pose = first_vive_pose
ee_init_pose = robot_tcp_at_startup  # 已知的机器人 TCP 位姿

# 计算对齐变换
vive2robot = ee_init_pose @ inv(vive_init_pose)

# 后续每帧
action_pose = ee_init_pose @ inv(vive_init_pose) @ current_vive_pose
```

**验证**：当 `current_vive_pose == vive_init_pose` 时：
```
action_pose = ee_init_pose @ inv(vive_init_pose) @ vive_init_pose
            = ee_init_pose @ I
            = ee_init_pose ✓
```

## 6. 数据格式

### 6.1 四元数格式

| 来源 | 格式 | 说明 |
|------|------|------|
| pysurvive 原始输出 | `[qw, qx, qy, qz]` | W 在前 |
| Rerun 可视化 | `[qx, qy, qz, qw]` | W 在后 |
| 我们的输出 | `[qw, qx, qy, qz]` | 与 pysurvive 一致 |

### 6.2 位置单位

- 所有位置值单位为 **米 (m)**

## 7. 标定最佳实践

1. **确保 Tracker 静止**：标定时 Tracker 必须完全静止
2. **清晰视线**：Tracker 与两个 Lighthouse 之间无遮挡
3. **稳定放置**：使用三脚架或固定支架
4. **足够时间**：至少标定 30 秒以上
5. **检查绿灯**：
   - Lighthouse：绿灯常亮表示正常工作
   - Tracker：绿灯常亮表示已配对并跟踪中

## 8. 参考资料

- libsurvive 源码：https://github.com/cntools/libsurvive
- HTC Vive Tracker 开发者指南
- `poser.c` - 坐标系建立逻辑
- `driver_global_scene_solver.c` - 全局场景求解器
- `poser_mpfit.c` - MPFit 非线性优化

