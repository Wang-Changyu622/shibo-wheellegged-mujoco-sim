# 学习控制参考项目与下一阶段路线

核查日期：2026-11-20。本文补充 [传统控制与交互项目审阅](additional_projects.md)，服务于“现有台阶效果有限，下一阶段是否引入学习”的决策。以下结论来自原仓库 README、许可证与具体配置/部署代码的静态审阅；没有安装、运行或训练这些外部项目。上游演示、README 自述和本项目实测结果严格分开。固定提交是本次核查时默认分支的 HEAD，不是经过本机验证的依赖锁。

用户最新决定暂不开展学习，本文保留为后续参考。若未来恢复，建议先完成台阶失败机理与硬件参数验证，再比较 **现有控制、受限残差策略、端到端 PPO**。强化学习可以探索接触时机和动作配合，不能消除轮径、几何行程、电压、舵机速度与转矩限制。

## 1. 六个已核验的主要参考

| 项目 / 默认分支 / 固定提交 | 实际提供什么 | 借鉴价值与不能照搬的地方 | 根许可证 |
| --- | --- | --- | --- |
| [Wheel-Legged-Gym](https://github.com/clearlab-sustech/Wheel-Legged-Gym/tree/c354431e5633eb98d9663fa3fbe444201d2de383) / `master` / `c354431e5633eb98d9663fa3fbe444201d2de383` | Isaac Gym 轮腿 PPO 环境；普通关节动作与 VMC 动作两类任务；地形课程、历史观测、特权 critic、参数随机化 | 直接相关的轮腿学习参考。6 动作、腿长和执行器能力与本车不同；README 的开/闭链统一表示不能代替本车真实 MuJoCo 闭链验证 | [BSD-3-Clause](https://github.com/clearlab-sustech/Wheel-Legged-Gym/blob/c354431e5633eb98d9663fa3fbe444201d2de383/LICENSE) |
| [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie/tree/d7895789a601943f78b3b9bcdc4def7b444f9913) / `main` / `d7895789a601943f78b3b9bcdc4def7b444f9913` | 双轮机器人速度跟踪 PPO；GPU 训练环境与 CPU MuJoCo/ONNX 播放；速度及推扰课程 | 接近本项目的任务组织，但为 **4 个腿位置＋2 个轮速度动作**；本车为 **2 个髋位置＋2 个轮 Uq**。该 README 明确实机展示使用 MPC，RL 速度策略实机部署仍属计划 | [Apache-2.0](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/LICENSE) |
| [mjlab](https://github.com/mujocolab/mjlab/tree/5e3b2e6cf1f00b37515a6658d1972e47e6059da0) / `main` / `5e3b2e6cf1f00b37515a6658d1972e47e6059da0` | PyTorch＋MuJoCo Warp＋RSL-RL；按观测、动作、奖励、事件、终止、课程与指标拆分环境 | 优先评估的训练框架候选，需先验证本车闭链与执行器语义。其通用速度任务给 actor 地形扫描和基座线速度，不能把默认配置直接称为本车可部署的纯本体感知策略 | [Apache-2.0](https://github.com/mujocolab/mjlab/blob/5e3b2e6cf1f00b37515a6658d1972e47e6059da0/LICENSE) |
| [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground/tree/8bac7b2c703ff61846ced7b00693cedbd5dbd7d8) / `main` / `8bac7b2c703ff61846ced7b00693cedbd5dbd7d8` | 当前主线是 PyTorch、reaf-sim、MuJoCo Warp，含 RSL-RL、仿真/实机接口、数据记录和辨识工作流 | 官方生态参考与框架备选；当前主线示例不能证明本车已受支持。旧 RSS 2025 的 JAX/MJX 环境在同一提交的 **`rss2025/` 目录**，不能混用新旧教程和包版本 | [Apache-2.0](https://github.com/google-deepmind/mujoco_playground/blob/8bac7b2c703ff61846ced7b00693cedbd5dbd7d8/LICENSE) |
| [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/8fa29acc6fd1910c3d9659eef6310bdd301cde0a) / `master` / `8fa29acc6fd1910c3d9659eef6310bdd301cde0a` | 足式机器人的 PPO 环境、粗糙地形、观测噪声、推扰和执行器模型等经典训练流程 | 适合学习实验结构；默认 12 动作与米尺度地形不适合直接搬到微型双轮车。README 已说明该仓库维护有限、后续迁移 Isaac Lab，旧 Isaac Gym 安装链不作为本项目首选 | [BSD-3-Clause](https://github.com/leggedrobotics/legged_gym/blob/8fa29acc6fd1910c3d9659eef6310bdd301cde0a/LICENSE) |
| [rl_sar](https://github.com/fan-ziqi/rl_sar/tree/376d42c9b128f963ab08579762d5a216a976ce39) / `main` / `376d42c9b128f963ab08579762d5a216a976ce39` | 策略部署、MuJoCo/Gazebo sim-to-sim 与实机适配框架；关节顺序、轮/腿动作分流、控制状态机 | 主要参考部署契约，不是现成的本车训练环境。支持矩阵逐机器人不同：该 README 的 Go2W 有 MuJoCo 支持，Tita 没有；不能概括为所有轮腿都已支持。当前 README 说明 Python 版暂缓维护，主线 C++ | [Apache-2.0](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/LICENSE) |

许可证核验只覆盖所链接的根许可；模型、图片及转引资产可能有独立来源要求。本次没有将这些项目的源码或资产并入本仓库。

## 2. 从代码看，它们通常做什么

### 观测、动作和时序先定义清楚

Upkie 的 actor 使用腿关节位置、轮速度、姿态四元数、角速度、上一动作和命令；critic 额外读取基座真实线速度。腿动作映射到位置目标，轮动作映射到速度目标；物理步长 5 ms、decimation=4，对应 **50 Hz 策略、200 Hz 物理步**。这些是该固定版本配置的值，不是本车推荐的积分频率。见 [观测与动作，L55–125](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/src/mjlab_upkie/tasks/upkie_velocity_env_cfg.py#L55)、[物理配置，L86](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/src/mjlab_upkie/robot/upkie_constants.py#L86)、[decimation，L339](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/src/mjlab_upkie/tasks/upkie_velocity_env_cfg.py#L339)。CPU 播放重新构建同样顺序的输入，并提供延迟缓冲选项，见 [sim.py，L76](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/sim.py#L76)。

Wheel-Legged-Gym 的基础配置使用 6 动作、27 维 actor 单帧观测、5 帧历史及额外特权状态；基础物理步长 5 ms、decimation=2，即 100 Hz 策略。其 VMC 配置包含腿角、腿长和轮速度尺度，并设置 40 N 腿部前馈力。不能把这类腿部虚拟力直接发给本车 STS3032 的位置接口。见 [基础配置](https://github.com/clearlab-sustech/Wheel-Legged-Gym/blob/c354431e5633eb98d9663fa3fbe444201d2de383/wheel_legged_gym/envs/base/legged_robot_config.py#L35)、[VMC 配置](https://github.com/clearlab-sustech/Wheel-Legged-Gym/blob/c354431e5633eb98d9663fa3fbe444201d2de383/wheel_legged_gym/envs/wheel_legged_vmc/wheel_legged_vmc_config.py)。

通用框架并不保证观测可在实机获得。例如 mjlab 速度任务的默认 actor 含 `base_lin_vel` 和 `height_scan`，不能因变量名包含 IMU 就视为普通 IMU 可直接测得的线速度。需要显式替换为可实现的估计量，或者声明策略使用地形感知。见 [mjlab actor/critic 配置，L76–153](https://github.com/mujocolab/mjlab/blob/5e3b2e6cf1f00b37515a6658d1972e47e6059da0/src/mjlab/tasks/velocity/velocity_env_cfg.py#L76)。

### 课程、随机化和奖励有不同职责

Upkie 当前任务地面是平面；课程逐阶段增加目标速度和推扰强度，地形课程代码被注释。奖励包含线/角速度跟踪、直立、默认姿态和动作变化惩罚；这说明它是速度/抗扰模板，不能称为已经解决台阶。见 [奖励，L234](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/src/mjlab_upkie/tasks/upkie_velocity_env_cfg.py#L234)、[课程，L297–323](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/src/mjlab_upkie/tasks/upkie_velocity_env_cfg.py#L297)。

Wheel-Legged-Gym 则包含多类地形、速度课程、质量/惯量/摩擦/质心/增益/延迟随机化。范围是其平台设定，例如基座加重可达数 kg、COM 偏移达数 cm；对本车约 0.318 kg 的先验模型必须重新定尺度。legged_gym 同样提供地形课程和推扰，但基类并非所有随机化都默认开启。见 [轮腿基础配置](https://github.com/clearlab-sustech/Wheel-Legged-Gym/blob/c354431e5633eb98d9663fa3fbe444201d2de383/wheel_legged_gym/envs/base/legged_robot_config.py)、[legged_gym 基础配置，L43、L121](https://github.com/leggedrobotics/legged_gym/blob/8fa29acc6fd1910c3d9659eef6310bdd301cde0a/legged_gym/envs/base/legged_robot_config.py#L43)。

课程决定练习顺序；随机化扩大参数覆盖；奖励提供学习信号；最终任务成功判据用于评估。四者不能混为一谈。奖励上升不等于两轮上台并回稳，更不等于实机性能已经提升。

### 部署是单独的一层验证

rl_sar 显式处理 `wheel_indices`、默认关节位置与动作尺度，分别生成位置和速度目标，再交给机器人接口。它提示本项目需要把 **动作单位、关节顺序、符号、限幅、控制周期和归一化统计量** 随策略一起保存。其位置/速度输出不能直接代替本车 Uq 驱动；必须使用本车相同的电压—速度—转矩代理和舵机约束。见 [rl_sdk.cpp，L256–271](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp#L256)、[发送位置/速度命令，L589–607](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp#L589)。

## 3. 训练框架选择有一个当前版本门槛

本项目目前冻结的 CPU MuJoCo 3.17 环境，应继续承担已有结果的复现。不能把下面任一仓库的最新依赖直接装进现有 `.venv`：

- mjlab 此固定提交的 `pyproject.toml` 要求 `mujoco-core[rendering]~=4.0.1`、`mujoco-warp~=4.0.1`、`torch>=2.14.0`、`rsl-rl-lib==5.5.2`。见 [精确依赖，L32–49](https://github.com/mujocolab/mjlab/blob/5e3b2e6cf1f00b37515a6658d1972e47e6059da0/pyproject.toml#L32)。
- Playground 此固定提交版本为 1.0.1，要求 Python 3.12–3.13、`mujoco[sysid]>=4.0.0`、`mujoco_warp>=4.0.0`，其 RL extra 固定 RSL-RL 5.4.2。见 [依赖，L1–35](https://github.com/google-deepmind/mujoco_playground/blob/8bac7b2c703ff61846ced7b00693cedbd5dbd7d8/pyproject.toml#L1)。当前训练入口实际导入 PyTorch、Warp、RSL-RL，见 [train.py，L15–42](https://github.com/google-deepmind/mujoco_playground/blob/8bac7b2c703ff61846ced7b00693cedbd5dbd7d8/mujoco_playground/rl/rsl_rl/train.py#L15)。旧 JAX 版本的安装入口另见 [rss2025/README](https://github.com/google-deepmind/mujoco_playground/blob/8bac7b2c703ff61846ced7b00693cedbd5dbd7d8/rss2025/README.md)。

这些是审阅时的版本事实，不是已经验证的新安装方案。先在独立环境锁版本、检查 Python/CUDA/驱动与包解析，再决定采用哪一个；也可以评估兼容旧 MuJoCo 的历史版本，但应固定提交并重新核查 API，不能仅凭旧教程猜版本。

官方文档说明 MuJoCo Warp 针对 NVIDIA 并行吞吐优化，单世界低延迟通常仍适合 CPU。设备转换支持检查不能替代数值一致性测试；接触和约束缓冲容量还会影响显存与正确性。见 [官方 Warp 用途、兼容性与容量说明](https://mujoco.readthedocs.io/en/stable/mjwarp/index.html)。MJX 同时有 JAX/Warp 实现，二者的自动微分与数据更新方式不同，不能把名称当成相同后端，见 [官方 MJX 实现说明](https://mujoco.readthedocs.io/en/stable/mjx.html#mjx-implementations)。

针对本机 RTX 4050 Laptop、约 6141 MiB 显存，建议按下列顺序立项，**尚未执行**：

1. **同模型检查**：自由基座、两条四杆 `connect` 约束、关节分支、质量/惯量、全部碰撞和四执行器输入单位保持一致；先确认所选后端支持实际模型特性。不得通过删除闭链、关闭关键碰撞或换成无限力矩执行器来制造兼容。
2. **CPU 升级差异单独测**：先比较原 CPU 3.17 与候选 CPU 版本，再比较候选 CPU 与 GPU，避免把版本变化和精度变化混在一起。
3. **分阶段数值检查**：匹配初态/控制输入，先比较运动学、质量矩阵、实际执行器力，再检查约束残差、接触和加速度；测试静止、轮电压输入、腿动作、台阶接触切换、落地。接触按 geom 对与空间位置匹配，不依赖接触数组顺序。短时差异与任务指标分别报告，不要求长时间接触轨迹逐点一致。
4. **小批量基准**：从 1 世界正确性开始，再做 **16→64→128** 世界的编译耗时、峰值显存、物理步吞吐、策略步吞吐和接触/约束溢出检查。冷启动与预热后分开测；只有留有显存余量才增加批量。上游 2048/4096 环境示例不代表这台 6 GB 机器能承受本模型同等规模。
5. **保留回退路径**：GPU 验证未通过时，继续 CPU 小规模策略接口与实验流程验证。训练中可以关闭画面，但不改变物理；可视网格与碰撞代理分工要与当前基线一致。

## 4. 适合本车的学习接口

以下是设计建议，尚未实现策略、训练权重或训练成功结果。完整实验约束沿用 [research_plan.md](research_plan.md)，硬件边界沿用 [hardware_parameters.md](hardware_parameters.md)。

| 决策 | 建议第一版 | 原因与边界 |
| --- | --- | --- |
| 低层控制 | 保留当前 LQR、位置舵机、轮电压限幅、接触相位及停车语义 | 先研究动作协调带来的收益，保留可解释对照；RL 不直接写自由基座状态或外力 |
| 策略动作 | 有界左右髋目标修正与接近速度/加速度调节；经共同限位和变化率限制 | 本车两个髋并非四个独立力矩关节；动作空间应反映真实接口 |
| 更强动作接口 | 仅在上述方案验证后，另做有界轮 Uq 残差实验；与端到端四动作 PPO 对照 | 残差与基线相加后统一执行电压/执行器限制，不能分别限幅后超出总边界 |
| 策略频率 | 先比较 50 / 100 Hz 的慢层；保持当前低层 0.5 ms 控制/积分基线 | 这是待测设计，不承诺最佳频率。物理步、低层控制和策略周期分别记录，不能为提速直接换成上游 5 ms 积分 |
| actor 输入 | 现有 IMU 姿态或重力投影、角速度、髋位置/速度、轮编码器速度、命令、上一动作、短历史及可部署控制器状态 | 当前传感器噪声与时延待标定。无真实硬件量对应的控制器状态需标注来源，防止间接泄露真值 |
| critic 输入 | 可额外读取仿真线速度、接触力、地形高度和随机化参数 | 训练特权信息与部署 actor 输入分开；策略导出时不应悄悄保留 critic 真值 |
| 地形信息 | 第一版明确区分“已知地形预览”和“无预览接触反应” | 没有前视传感器时，未接触台阶的位置通常不能从本体观测唯一恢复；历史策略不能凭空知道尚未观测的边缘 |
| 状态与重置 | 独立 Gymnasium 风格 `reset/step` 适配层，复用现有动力学与执行器配置 | 每回合重置积分器、LQR 参考、跳跃相位、滤波/延迟队列和历史；失败与时间截断分开，禁止回合间信息泄漏 |
| 策略部署包 | 权重＋输入顺序/单位＋动作映射＋归一化统计＋版本/模型哈希＋控制周期 | 用同一动作序列比较训练环境、CPU viewer、ROS 入口的实际输入输出，再接实机 |

残差控制不天然保证稳定。应限制变化率、接管范围和失效处理，并以与基线相同的扰动/初值检验；高频控制不能只在策略更新时执行一次。原真值 LQR 基线可以先用于“可行性上限”实验，但如果它使用实机不可得的全局位置/速度，残差系统整体同样属于真值控制，不能只约束 actor 就称为可部署。

## 5. 台阶课程与公平评价

先用当前诊断得到的 **可通过、边界、失败** 三类台阶作为开发集起点，按毫米数与轮半径比共同报告。建议顺序是平衡及停车 → 正反向变速 → 单个低台阶及下台 → 左右非对称支撑/单边桥 → 连续台阶 → 单独的起跳/落地任务。高度、边缘形状、接近速度、横向偏移与摩擦逐步增加；未知地形测试集在调参前冻结。具体高度不能直接复制大轮腿项目的米尺度地形。

奖励设计需要鼓励向平台推进、双轮形成平台支撑、上台后持续回稳，同时记录碰撞冲量、滑移、动作变化、实际能量代理、饱和时间和姿态。不要用过强的固定站高/直立惩罚把必要的蹲伸、瞬时俯仰和离地全部惩罚掉。跳跃另设明确相位和成功事件；仅增加竖直速度奖励可能产生抖动、撞地或利用接触误差。累计奖励不作为任务验收指标。

建议对照矩阵如下，所有项目均为后续实验：

| 对照 | 控制方法 | 必须保持一致 |
| --- | --- | --- |
| C0 | 当前冻结的传统控制 | 相同硬件 profile、初态、地形、时间预算与成功判据 |
| C1 | 同一传统控制＋慢层腿/速度残差 PPO | 与 C0 相同信息；如提供地形预览，C0 也须有可用的预览基线 |
| C2 | C1＋有界轮电压残差 | 相同总执行器包络；额外复杂度与饱和情况单独报告 |
| C3 | 直接四动作 PPO | 相同模型、观测、限幅；给予合理且记录在案的调参预算 |
| 消融 | 去掉历史、去掉课程、去掉执行器随机化、真值/可测观测切换 | 一次改变一个关键因素，不靠给新方法额外真值构造收益 |

初期至少比较多个独立训练种子，建议从 3 个开始估计方差，再根据效应与预算决定正式数量。开发种子和测试种子分开；同一确定性初态的重复播放不能当成独立成功率样本。保存每个 checkpoint 的选择规则，测试集不能用于选最好的训练种子。

最终至少报告：各高度通过率及置信区间、成功后回稳率、过阶时间、pitch/roll 峰值、冲击与滑移、实际电压/力矩饱和占比、跌倒/卡住/单轮支撑等失败类别、计算延迟；条件于成功的指标必须标明，不删去失败试次。轮径、电压、峰值与连续舵机包络不得在不同算法间更换。

## 6. 形成论文问题前的结束条件

本阶段研究主线候选是：**在低成本位置舵机与轮电压约束下，学习残差能否扩大接触切换任务的可靠成功域，并保留可解释低层控制？** 这目前是待检验问题，不是已成立的新颖贡献或 RAL 水平证明。

针对用户提出的“现有台阶太低，要挑战更高或连续台阶”，建议先建立高台阶可行域，再开展连续级数课程；以下仍是设计建议，尚未验证更高台阶能力。上游地形不能按米尺度直接移植，应共同记录单级高度 h、轮直径 D、h/D、踏面深度 d、级数 N、入口助跑长度和顶部平台长度。当前两轮同轴，没有可用于归一化踏面深度的前后轮轴距；建议同时报告 d/D，以及踏面相对于机体前后碰撞包络、可达接触区域的尺寸关系，不能把左右轮距误当作纵向轴距。高度、踏面深度和连续级数分别扫描，避免只提高台阶后把“触边卡住”“轮已上台但机体碰撞”“没有足够空间回稳”混成一个失败原因。

先在相同 2S 电压、髋额定包络和受限腿行程下，测单级台阶的接近速度—高度—踏面深度可行域，识别几何触达、轮电压饱和、舵机速度与落地恢复的限制。再从单级扩展为双级、三级和更多级：先固定已验证的单级高度与较宽踏面，仅增加级数；通过后再缩短踏面、改变级高和平台长度，并将未见组合留作测试集。双轮同轴和有限腿行程使逐级支撑转换不同于前后多轮平台，不能由大轮腿的演示推定本车能力。没有前视传感器或明确地形先验时，历史本体观测通常不能唯一确定尚未接触的下一台阶；“已知楼梯预览”与“无预览接触反应”应分别设实验，不能用仿真地形真值暗中弥补。

连续台阶评分建议使用固定宽度走廊、各级边缘与踏面区域、逐轮支撑事件和终点回稳窗，逐级核验真实进度。仅看最终 x 或高度会让绕侧面、跨过检查点或单轮悬空被误计为成功；绕行、机体撑地、未按任务要求形成踏面支撑应记录为相应失败类别。若另设跨级跳跃任务，应单独声明允许的支撑序列并评分，不能把跳过中间踏面的轨迹计作逐级通过。报告完整楼梯通过率、实际完成级数、首个失败级及原因，以及顶部平台回稳率。

进入较大训练前应已有：上述可复现失败边界、执行器速度/延迟模型的证据或明确假设、闭链后端一致性报告、无真值泄漏的观测契约、独立于奖励的任务评估器，以及本机吞吐/显存基准。端到端 PPO 和真值上限作为对照保留。额外加速度计优化继续放在独立后续阶段，避免同时改变传感器与控制方法而无法解释收益来源。
