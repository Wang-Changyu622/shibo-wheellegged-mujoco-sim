# 当前代码框架

2026-11-20。当前框架是 **CAD 闭链模型 + MuJoCo + LQR/位置舵机协调 + 跳跃监督器 + 可视化/ROS + 验收诊断**。按最新要求，本轮只收尾审查、诊断和框架说明，**不搭建学习环境、不训练，也不实施计划中的核心重构**。高台阶、连续台阶是后续非学习控制工作的优先项，当前尚未实现通用能力。

## 实际文件树

同层文件可在一行并列；网格、文档与运行产物折叠显示。`assets/cad/` 是安装后存在、被 Git 忽略的下载缓存。

```text
.
├── pyproject.toml、README.md、LICENSE、THIRD_PARTY_NOTICES.md
├── .github/workflows/simulation.yml
├── src/wheellegged/
│   ├── __init__.py、cli.py
│   ├── hardware.py、mass_properties.py、model.py
│   ├── state.py、lqr.py、control.py、jump.py
│   └── evaluate.py、viewer.py、render.py
├── ros2/
│   ├── README.md
│   └── wheellegged_ros/
│       ├── package.xml、setup.py、setup.cfg
│       ├── resource/wheellegged_ros
│       ├── launch/simulation.launch.py
│       └── wheellegged_ros/__init__.py、sim_node.py
├── config/hardware_parameters.json
├── models/README.md、cad_mass_properties.json
├── assets/cad/                       # 本地缓存，不随 Git 提交
│   ├── mushibot.xml、cad-manifest.json
│   ├── reference-parameters.json、THIRD_PARTY_NOTICES.md
│   └── meshes/                      # part_*.obj、contact_*.obj
├── scripts/
│   ├── setup.sh、asset-lock.json、fetch_assets.py
│   ├── export_cad_mass_properties.py、check_dynamics.py
│   ├── viewer_smoke.py、leg_motion_smoke.py
│   ├── ros2_run.sh、ros2_smoke.py、ros2_launch_smoke.py
│   └── step_diagnostics.py、review_checks.py、render_review_figures.py
├── tests/
│   ├── test_model.py、test_mass_properties.py、test_state.py
│   ├── test_command_limits.py、test_jump.py、test_control.py
│   └── test_viewer_input.py、test_leg_commands.py、test_leg_motion.py
├── docs/                            # 来源、操作、审查、计划；evidence/ 与 media/
├── outputs/                         # 本地完整轨迹、视频和诊断；Git 忽略
└── artifacts/                       # 本地构建及运行验证产物；Git 忽略
```

## 模块职责与数据流

| 当前模块 | 实际职责 |
| --- | --- |
| [hardware.py](../src/wheellegged/hardware.py) / [mass_properties.py](../src/wheellegged/mass_properties.py) | 读取两个硬件 profile；依据 STEP 数值表修正组件质量、质心和惯量 |
| [model.py](../src/wheellegged/model.py) | 四杆解算、MJCF/地形/传感器组合、自由基座初始化、闭链和接触查询 |
| [state.py](../src/wheellegged/state.py) / [lqr.py](../src/wheellegged/lqr.py) | 提取 MuJoCo 真值观测；利用实际质量矩阵设计局部离散 LQR，提供七个髋姿态增益 |
| [control.py](../src/wheellegged/control.py) / [jump.py](../src/wheellegged/jump.py) | Command 校验、参考限速、轮电压/偏航、左右腿/横滚协调；接触驱动跳跃及拒绝/恢复事件 |
| [evaluate.py](../src/wheellegged/evaluate.py) | 固定命令表、物理 rollout、逐步轨迹与独立验收 |
| [viewer.py](../src/wheellegged/viewer.py) | GLFW 按键/焦点、交互 Session、HUD；另提供 ROS passive 窗口 helper |
| [sim_node.py](../ros2/wheellegged_ros/wheellegged_ros/sim_node.py) | ROS 命令、watchdog、timer 步进、时间戳、遥测与 TF |
| [render.py](../src/wheellegged/render.py) | 从保存的状态重放视频、绘制曲线；不参与控制和评分 |

初始化链路：固定 CAD 缓存 + 硬件 JSON + 质量表 → `model.load_model` → `balanced_data` → `BalanceController`/LQR。原始 STEP 只用于离线参数导出；日常仿真读取已有数值表，不需要 CAD 环境，也不修改源 XML。

运行链路：固定命令 / GLFW / ROS → `Command` → `BalanceController.update`（调用 `observe` 和 `JumpSupervisor`）→ 四路 `ctrl` → `mj_step` → `mj_forward` → 下一步观测及记录/显示。三个入口分别由 `run_scenario`、`_Session.step`、`SimulationNode._advance` 持有循环；**目前没有统一 Simulation 类**。显式 reset 和离线视频重放不属于闭环运动控制。

## 四执行器与当前时序

模型为自由基座 + 左右各四个铰链 + 两个闭链约束，`nq=15 / nv=14 / nu=4`；被动关节不是额外输入。代码按名字索引执行器。

| 执行器名称 | 输入语义 | `hardware_2s` 命令范围 | 力矩上限 |
| --- | --- | --- | --- |
| `L_wheel_voltage`、`R_wheel_voltage` | 左右轮 q 轴电压，V | 各 `±3.7 V` | 各 `±0.0588 N·m` 外层代理上限 |
| `L_servo`、`R_servo` | 左右髋位置目标，rad | 各 `[-0.30, 0.0]` | 各 `±0.14709975 N·m` |

轮电压不是直接力矩，髋位置伺服也不是足端力/VMC 接口。更负的髋角表示伸腿；独立 `left_hip_angle/right_hip_angle` 为 `None` 时分别继承共同 `hip_angle`。请求、限速/反馈后的舵机输入和实际髋角是不同量；窗口与独立腿专项证据可区分这些层，通用 evaluate 日志仍遗漏左右请求和部分参考，见[代码审查](code_review_next_stage.md)。跳跃暂时对称接管双腿，回稳后恢复持续保留的左右请求。参数仍是待标定工程代理，见 [硬件说明](hardware_parameters.md)。

默认物理与反馈控制均为 **0.5 ms（仿真时间 2 kHz）**；改变 `run --dt` 会同时改变控制频率和 LQR 离散化。交互窗口固定 0.5 ms、显示目标约 60 Hz。ROS 默认每次 timer 回调执行四个子步，目标实时率为 1 时约 2 ms 回调，内部控制仍为 2 kHz；遥测为 100 Hz。CPU 不足时实时率降低，不跳过物理步。目前没有独立策略周期。

## 实际运行入口

[pyproject.toml](../pyproject.toml) 将 `wheellegged` 映射到 [cli.py](../src/wheellegged/cli.py)。安装后在仓库根目录运行：

```bash
.venv/bin/wheellegged run all --output outputs/new-run --video
.venv/bin/wheellegged view --terrain flat
.venv/bin/wheellegged view --scenario step --auto
bash scripts/ros2_run.sh --ros-args -p use_sim_time:=true -p viewer:=true
# colcon 安装后：
ros2 launch wheellegged_ros simulation.launch.py profile:=hardware_2s viewer:=true
```

键盘：W/S/A/D 按住运动、释放目标归零；Q/Z 左腿、E/C 右腿、T/G 双腿伸缩，H 默认高度，J 单次跳跃，X 停车，1–5 自动演示，M 手动/自动，R reset，P/Space 暂停，F 相机。见 [可视化说明](visualization.md)。ROS 输入 `/cmd_vel`、`/leg_command`、`/jump`，输出 `/clock`、`/joint_states`、`/imu`、`/odom`、动态/静态 TF；速度命令超时 0.5 s 后归零而腿目标保留。ROS 窗口由主题驱动，构建与参数见 [ROS README](../ros2/README.md)。

## 诊断分层与完成边界

环境/资产层为 [setup.sh](../scripts/setup.sh)、[fetch_assets.py](../scripts/fetch_assets.py) 和锁定清单；离线 CAD 层为 [export_cad_mass_properties.py](../scripts/export_cad_mass_properties.py)；动力学/接口验收层为 `check_dynamics.py`、`viewer_smoke.py`、`leg_motion_smoke.py` 及两个 ROS smoke 脚本。`tests/` 分别覆盖模型、质量、坐标、命令/跳跃、真实物理任务和窗口输入，现有 [140 项测试通过](evidence/pytest_with_leg_controls.txt)。

新增诊断保持在核心之外：[step_diagnostics.py](../scripts/step_diagnostics.py) 扫描 12 个确定性单级台阶组合并区分任务完成与运动质量；[review_checks.py](../scripts/review_checks.py) 记录隔离审查观测，默认无 ROS、`--ros` 可选，**不是总体任务 PASS**；[render_review_figures.py](../scripts/render_review_figures.py) 从既有轨迹生成图像。专项脚本读取实际地形，并不表示通用评价器已经完成地形接口重构。

**已有能力**是指定条件下的平衡、前后移动、6 mm 单边桥/单台阶、微跳、独立左右腿高度，以及可视化/ROS/可追溯诊断。8/10 mm 指定条件的失败已保留在 [台阶诊断](step_diagnostics.md)；高台阶策略和连续阶场景尚未实现。

**尚无且本轮不启动**：Gym/Gymnasium 环境、训练入口、奖励/curriculum、策略权重/ONNX、学习 rollout、actor/critic 观测接口、硬件估计器注入、统一运行内核和独立物理/控制/策略时钟。当前高低层控制仍使用 MuJoCo 真值。已有 [审查](code_review_next_stage.md)、[学习计划](learning_control_plan.md) 和 [参考调查](learning_references.md) 只是后续方案，不应列为已经完成的代码模块。
