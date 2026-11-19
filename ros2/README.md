# ROS 2 适配器

`wheellegged_ros` 是薄适配层；建模、LQR 和运动状态机由同一个 `wheellegged` 核心提供。节点通过执行器控制驱动自由基座 MuJoCo 模型，没有直接改位姿的移动或跳跃。

## 本机运行

本项目当前 `.venv` 继承本机 RoboStack ROS 2 Humble 环境，已经能导入 `rclpy`。在项目根目录运行：

```bash
bash scripts/ros2_run.sh --ros-args -p use_sim_time:=true -p terrain:=flat -p profile:=hardware_2s -p viewer:=true
```

命令行发布工具需在 ROS 2 环境中运行，例如使用本机环境的 `bin/ros2`。持续发布前进速度：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}'
ros2 topic pub --once /jump std_msgs/msg/Empty '{}'
```

`linear.x` 单位为 m/s，`angular.z` 为 rad/s。负速度后退。命令超过 0.5 秒仿真时间没有更新后，速度和偏航率目标归零；控制器仍保持平衡。NaN/Inf 速度或偏航率命令被拒绝，也不会续期命令超时计时器。一次 `/jump` 消息触发一次请求，是否执行由核心跳跃状态机决定。

## 标准 colcon 构建

在包含 ROS 2、MuJoCo 和本项目核心的同一个 Python 环境中：

```bash
python -m pip install -e .
colcon build --base-paths ros2
source install/setup.bash
ros2 launch wheellegged_ros simulation.launch.py terrain:=flat profile:=hardware_2s viewer:=true
```

若采用 RoboStack，需先激活其环境以便构建器、Python 与共享库一致；不要混用系统 Python 和另一发行版的 `rclpy`。本次普通构建已在项目环境的 colcon-core 0.21.3、setuptools 79.0.1 下实测通过。`scripts/ros2_run.sh` 无需 colcon，可直接验证适配层。脚本可通过 `WHEELLEGGED_PYTHON` 指定兼容的 Python。

`profile` 默认 `hardware_2s`，采用当前硬件电压/力矩约束及组件质量映射；`reference` 只供历史模型复现，未知名称会在启动时拒绝。`viewer` 默认 `false`，便于服务器和自动测试；设为 `true` 时打开原生 MuJoCo 窗口，约 60 Hz 同步同一个 ROS 仿真状态，显示参数 profile、仿真时间与目标速度。控制仍从 `/cmd_vel`、`/jump` 输入。相机支持原生鼠标旋转和缩放；带键盘控制、场景切换和更多诊断的独立入口见主项目可视化说明。

关闭窗口只停止显示，ROS 仿真继续运行；Ctrl-C 停止节点并关闭窗口。节点等待此次创建的后台窗口线程结束再退出，避免 GLFW 清理与 daemon 渲染线程竞争。无图形会话时使用默认 `viewer:=false`。

## 接口和时间

| 主题 | 类型 | 约定 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 订阅前进速度与偏航率 |
| `/jump` | `std_msgs/msg/Empty` | 订阅单次跳跃请求 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布所有铰链/滑动关节 SI 状态，SensorData QoS |
| `/imu` | `sensor_msgs/msg/Imu` | 发布 `imu_link` 的理想 MuJoCo IMU，SensorData QoS |
| `/odom` | `nav_msgs/msg/Odometry` | 发布 `odom` 位姿与 `base_link` 速度 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 发布 MuJoCo 仿真时间 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom → base_link` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 从 MJCF site 读取 `base_link → imu_link` |

MuJoCo 的 `[w, x, y, z]` 正确转换为 ROS `[x, y, z, w]`。IMU 加速度为 MuJoCo accelerometer 的比力，包含重力方向的静止读数；没有把世界坐标的运动学加速度伪装成 IMU。IMU 杆臂来自模型 site，不假定与基座重合。协方差保留零值（未知），不是声称实物无噪声；odom 是仿真真值，尚未接入里程计估计器。

`odom.twist` 的参考点和坐标轴均为 `base_link`。MuJoCo `mj_objectVelocity` 使用 `mjOBJ_XBODY`；`mjOBJ_BODY` 对应惯性质心及其主轴，当 CAD 惯性坐标与机器人坐标不重合时不能混用。验收以 body 原点雅可比和自由关节运动学独立计算期望速度，不复用发布器的速度接口。

物理步进默认 **0.5 ms**，使用 steady wall clock，消息时间使用 MuJoCo `data.time`，因此开启 `use_sim_time` 后不会等待自身 `/clock` 而死锁。`realtime_factor` 为目标速率，CPU 不足时仿真会慢于目标；不会跳过物理步。每个物理步完成后重新 forward，保证下一步控制观测与输出传感器均对应当前状态。这个调用顺序与核心无界面仿真一致。

可用节点参数：`terrain`、`profile`、`viewer`、`physics_dt`、`realtime_factor`、`publish_rate`、`command_timeout`、`hip_angle`、`steps_per_callback`。launch 文件暴露 `terrain`、`profile`、`viewer`、`realtime_factor`；其他参数可用直接节点入口设置。参数在启动时读取，运行期更改不受支持。

## 本地 DDS 集成验证

```bash
PYTHONPATH=src:ros2/wheellegged_ros .venv/bin/python scripts/ros2_smoke.py
.venv/bin/python scripts/ros2_launch_smoke.py
DISPLAY=:0 .venv/bin/python scripts/ros2_launch_smoke.py --viewer
```

测试在独立默认 `ROS_DOMAIN_ID=73` 上运行真实 rclpy 发布订阅和真实 MuJoCo 模型。默认报告为 `artifacts/ros2/smoke.json`，记录实际步长、MuJoCo 版本和关键源码 SHA-256；失败会覆盖旧的成功状态，避免误读历史报告。

已覆盖以下接口范围：

- 正负速度命令通过 DDS 到达控制器，断流后参考速度和偏航率归零。
- NaN 线速度及 Inf 偏航率通过 DDS 到达订阅回调后被拒绝，原命令和超时计时器均保持不变。
- 8 个真实铰链关节名称完整、位置/速度/力矩数组等长、数值有限，运动期间轮关节读数变化，防止空数组或全零占位读数。MuJoCo 的关节枚举与 NumPy 整数先转换成 Python `int` 后比较，规避本机绑定中导致所有关节被过滤的比较问题。
- `/clock` 严格递增，`use_sim_time` 下仍能启动；每条 IMU/odom/动态 TF 消息都与相同时间戳的 MuJoCo 快照逐字段比对，包括 xyzw 四元数、姿态、比力与局部速度。
- 动态运行结束后，单独构造倾斜基座和六维非零速度的运动学夹具，通过 DDS 核验 `base_link` twist；夹具区分普通 body 轴与惯性主轴，随后恢复原仿真状态，不将其计作机器人运动任务。
- 用 transient-local QoS 读取先于订阅者发布的静态 TF，验证 `base_link → imu_link` 与 MJCF IMU site 的位置和姿态完全一致。
- 单次 `/jump` 消息恰好向核心转发一次请求；传感器和 TF 持续发布。

`ros2_launch_smoke.py` 在独立 `ROS_DOMAIN_ID=74` 上执行普通 colcon 构建，通过真正的 `ros2 launch` 启动已安装节点，再由另一个进程订阅 `/clock`，最后发送 SIGINT，同时确认 launch 返回 0 且仿真子进程日志记录 clean exit。还检查已安装节点拒绝未知 profile。`--viewer` 使用 domain 75，打开真实窗口，通过外部 `/cmd_vel` 驱动并检查 odom 位置变化，再完整关闭。默认从当前 Python 的基础环境或已激活的 ROS 路径定位依赖；也可用 `--ros-prefix` 指定 ROS 安装前缀。构建目录和日志写入 `artifacts/ros2`，不会作为源码交付。

当前 `hardware_2s` 模型总质量约 0.317649 kg。DDS 测试运行 2.8 秒仿真时间，5 类动态消息各收到 279 条，静态 TF 收到 1 条；独立无头 launch 测试收到 26 条递增时钟，到达 0.25 秒。真实窗口测试达到 1.0 秒，收到 101 条时钟、100 条 odom，外部命令使基座 x 变化约 0.02684 m。可提交的小型报告见 [DDS 证据](../docs/evidence/ros2_smoke.json)、[构建/launch 证据](../docs/evidence/ros2_launch.json) 和 [原生窗口/ROS 命令证据](../docs/evidence/ros2_viewer_launch.json)，其中 SHA-256 对应本次实际执行的代码。旧 profile、修正前坐标实现的历史报告保存在 `docs/evidence/reference_v2/ros2_*.json`，仅供溯源，不能作为当前帧正确性的证据。

该测试只证明适配层数据流，不验证硬件通信、真实传感器噪声、极端消息频率、网络丢包恢复或多个仿真节点共用绝对主题的场景。地形通过率、落地稳定性、模型闭环误差由核心仿真基准另行验证。
