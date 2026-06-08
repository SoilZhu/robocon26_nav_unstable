# Robocon 26 顺序路径点导航系统 (ROS 2 Humble)

> ## 本项目由 Gemini Vibe 而成，未经测试，仅提供思路验证

本项目为 Robocon 26 参赛机器人开发的顺序路径点导航与控制系统，包含传感器静态 TF 广播解耦、下位机串口通信模块以及基于状态机的路径巡航与色度位置校准算法。

## 1. 项目目录结构

```text
robocon26/
└── src/
    ├── livox_ros_driver2         # Livox 激光雷达驱动包
    ├── point_lio                 # 激光雷达惯性里程计 (发布 camera_init -> aft_mapped 变换)
    ├── rm_serial_driver          # 串口通信包 (底盘速度与高度控制收发、色度状态发布)
    ├── robocon_tf_broadcaster    # 静态 TF 广播包 (管理雷达与色度传感器安装偏置)
    ├── robocon_waypoints         # 路径点数据包 (管理导航路径与到位参考坐标参数)
    ├── robocon_gazebo            # Gazebo 仿真原型验证包 (Mock 硬件外设与 TF 坐标发布)
    └── robocon_nav               # 顺序路径点导航包 (直线控制速度发生器与坐标实时校准)
```

---

## 2. 外部开源包来源

本项目集成并修改了以下开源软件包：
- **livox_ros_driver2**：来自 [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- **point_lio**：来自 [SMBU-PolarBear-Robotics-Team/point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio)
- **rm_serial_driver**：来自 [rm-vision-archive/rm_serial_driver](https://github.com/rm-vision-archive/rm_serial_driver)

---

## 3. 核心包介绍

### 3.1 静态 TF 广播包 (`robocon_tf_broadcaster`)
- **作用**：解耦传感器物理安装位置。广播雷达相对于车中心，以及色度传感器相对于车中心的静态坐标变换，构成了完整的 TF 树：
  `camera_init (地图)` -> `aft_mapped (雷达)` -> `base_link (车中心)` -> `color_sensor_link (色度传感器)`
- **配置文件**：`src/robocon_tf_broadcaster/config/tf_offsets.yaml`。当机器人机械结构调整时，**只需修改此文件**，无需触碰导航与算法代码。

### 3.2 路径点数据包 (`robocon_waypoints`)
- **作用**：完全隔离策略数据与算法代码。集中管理机器人的行驶路径点列表（`waypoints.yaml`），方便战术组开发、调整或进行多套蓝方/红方策略路径的切换。
- **配置文件**：`src/robocon_waypoints/config/waypoints.yaml`。

### 3.3 顺序路径点导航包 (`robocon_nav`)
- **状态机控制逻辑**：
  1. `STATE_IDLE`：等待收到第一个雷达里程计数据，自动触发导航启动。
  2. `STATE_TRAVERSING`：平移至路径点。通过比例-减速曲线产生底盘速度命令，并在接近目标点时利用物理减速公式 $v \le \sqrt{2 \cdot a_{max} \cdot d_{err}}$ 确保机器人平稳减速，中途锁死偏航角。
  3. `STATE_ROTATING`：到达路径点坐标后，底盘平移速度清零，原地旋转对齐路径点的目标 Yaw 朝向。
  4. `STATE_HEIGHT_ADJUST`：朝向对齐后，发布升降高度命令 `/target_height`，保持底盘静止并等待单片机反馈 `height_reached == True` 信号或超时。
  5. `STATE_WAITING`：上述行为执行完毕后，原地等待配置的延时秒数。
  6. `STATE_FINISHED`：所有路径点依次巡航完毕，底盘停转，导航结束。
- **绝对坐标校准**：
  当订阅到 `/color_sensor_state == 1`（色度传感器扫到绝对参考线 $A$）时，导航节点会自动查找此时传感器在世界坐标系下的 X 坐标，推算得出累计漂移偏移量并实施动态 X 轴校准：
  $$X_{offset} = X_{ref\_x\_coordinate} - X_{sensor, camera\_init}$$
  $$X_{robot\_map} = X_{robot, camera\_init} + X_{offset}$$

### 3.4 串口通信包 (`rm_serial_driver`)
- **订阅主题**：订阅底盘运动指令 `/cmd_vel`（Twist）与目标机构高度 `/target_height`（Float32）。
- **发布主题**：发布到位信号 `/height_reached`（Bool）与传感器颜色状态 `/color_sensor_state`（UInt8）。
- **通信数据包定义**：
  - **发送数据包 (ROS 2 -> 单片机，共 19 字节)**：
    `[帧头 0xA5 (1B) | vx (4B) | vy (4B) | wz (4B) | target_height (4B) | CRC16 (2B)]`
  - **接收数据包 (单片机 -> ROS 2，共 9 字节)**：
    `[帧头 0x5A (1B) | height_reached (1B) | color_sensor_state (1B) | yaw (4B) | CRC16 (2B)]`

### 3.5 Gazebo 原型验证包 (`robocon_gazebo`)
- **作用**：完全解耦的仿真与测试环境，允许开发者在没有真实机器人硬件及 Point-LIO 雷达里程计运行的情况下，对 `robocon_nav` 逻辑进行快速闭合原型验证。
- **关键机制**：
  - **里程计 TF 桥接**：订阅 Gazebo 内置的底盘里程计 `/odom`，并根据配置的传感器物理偏置，动态广播 `camera_init` -> `aft_mapped` 变换，完全对齐真实 Point-LIO。
  - **高度机构 Mock**：订阅 `/target_height` 指令，仿真执行电机运动延迟，并在到达指定位置后自动向 `/height_reached` 话题广播到位信号。
  - **色度校准线 Mock**：计算色度传感器在仿真世界中的坐标，在穿过预设线位置时自动发布 `/color_sensor_state` 信号，完全闭环测试导航的绝对 X 轴标定与补偿逻辑。

---

## 4. 编译与运行指南

### 4.1 编译工作空间
在您的 ROS 2 终端中运行：
```bash
colcon build --symlink-install
```

### 4.2 运行指令

#### 选项 A：在真实机器人上运行
1. **启动串口通信**：
   ```bash
   ros2 launch rm_serial_driver serial_driver.launch.py
   ```
2. **启动雷达里程计与驱动**（根据您的实际雷达型号和 Point-LIO 启动）：
   ```bash
   ros2 launch point_lio mapping_mid360.launch.py
   ```
3. **启动导航系统**（会自动引入并加载静态 TF 广播）：
   ```bash
   ros2 launch robocon_nav nav.launch.py
   ```

#### 选项 B：在 Gazebo 仿真环境运行 (原型验证)
1. **启动仿真并级联拉起导航系统**（一键拉起 Gazebo、机器人描述、Mock 节点、静态 TF 广播与导航）：
   ```bash
   ros2 launch robocon_gazebo sim.launch.py
   ```
2. **（可选）仅运行仿真与 Mock 桥接节点**（不启动导航，用于单独调试）：
   ```bash
   ros2 launch robocon_gazebo sim.launch.py run_nav:=false
   ```

### 4.3 路径点与安装参数微调
- 路径点及校准参考参数配置路径：[waypoints.yaml](src/robocon_waypoints/config/waypoints.yaml)
- 雷达与传感器安装偏移配置路径：[tf_offsets.yaml](src/robocon_tf_broadcaster/config/tf_offsets.yaml)
