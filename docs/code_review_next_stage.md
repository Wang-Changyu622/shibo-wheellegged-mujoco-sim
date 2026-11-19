# 当前代码审查与学习控制前置工作

审查基线：`1cecec9f83af218b2b19618ab71cff5d529c2f99`，2026-11-20。只审阅和运行隔离探针，没有改动当前控制器、模型或测试；报告与复现脚本单独归档发布。基线 [完整测试记录](evidence/pytest_with_leg_controls.txt) 为 **140 passed**；闭链几何、默认任务、真实跳跃、独立腿动作、窗口输入与 ROS 已有相应证据。这些成果有效，但测试覆盖的是明确的有限任务域，不能由测试数量推出任意楼梯或实机迁移能力。

用户最新决定暂不开展学习。以下修复和扩展是审查建议，尚未实施；后续能力优先级为更高台阶与连续台阶。当前实际模块见[代码框架](code_architecture.md)。

下面区分三种结论：**缺陷**指现有接口遗漏或不一致；**已复现的边界行为**指名义测试未覆盖的控制行为；**研究前置条件**指已经公开说明的简化假设，不能倒过来称为实现错误。P1 表示应在下一轮学习/台阶实验前处理，P2 表示应在批量训练或部署之前处理。

## 七项优先发现

### 1. P1｜受阻时位置参考持续累积，零速度命令不会消除旧的位置误差

类型：已复现的边界行为。位置参考按期望速度持续积分，轮电压则在控制器最后统一限幅；两者之间没有受阻或饱和反馈。[参考积分](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/control.py#L101)、[位置误差进入 LQR](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/control.py#L142)、[执行器限幅](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/control.py#L175)。

触发与证据：用固定机器人观测模拟持续受阻，先请求 `+0.2 m/s` 三秒，再请求零速度一秒。速度参考已回到零，位置参考仍超前 **0.60 m**，两轮控制量仍为 **−3.7 V**。这是控制器契约夹具，**没有推进物理，也不是某个台阶失败的物理证明**。它确证“零速度参考”和“取消尚未完成的位移要求”当前不是同一个操作。

影响：机器人撞到难以越过的障碍时，旧误差会继续影响后续输出；仅用速度限幅不能保证人工停车或策略切换后的行为。轮电压的正负还取决于平衡状态，不能把上述负电压简单描述为持续向前推墙。

建议：明确区分轨迹跟踪模式与人工速度模式。为人工停车增加可检验的当前位置捕获/平滑参考重定位语义；对持续饱和或受阻添加有界参考管理。不要直接删掉位置环，也不要在未验证平衡瞬态前突然改写状态。验收应包括实际墙面/不可越障碍受阻、释放命令、再次起步以及恢复后的位移和姿态界限。

### 2. P1｜通用轨迹日志遗漏独立左右腿的命令与内部参考

类型：当前日志缺陷。[通用采样器](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L122) 记录 `command_hip_angle`，但没有记录 `left_hip_angle`、`right_hip_angle`、解析后的 `hip_targets`、差分参考或 `roll_reference`。

触发与证据：对同一个物理快照，分别传入默认 `(-0.2,-0.2)` 与独立请求 `(-0.3,0.0)`，`_sample` 返回的**全部字段相同**，记录的共同角都为 `-0.2`。现有 `hip`、`ctrl`、`actuator_force` 能说明实际状态和最终执行器输出，却无法唯一还原用户请求和反馈修正的区别。独立腿专项脚本有自己的额外记录，并不能代替通用记录契约。

建议：增加版本化记录结构，分别存放原始命令、限幅后的左右请求、slew 后参考、最终执行器输入、实际力矩、饱和标志、跳跃事件与拒绝原因。使用真实值而不是由共同角反推左右角。验收应证明不同请求不会发生日志碰撞，并能由记录重放同一控制输入序列。

### 3. P1｜减半物理步长时，控制频率和 LQR 增益也一起变化

类型：研究验证方法缺口，不是现有离散设计错误。控制器周期直接取 `model.opt.timestep`，[控制器初始化](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/control.py#L50)；LQR 也按该值离散化，[离散模型](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/lqr.py#L62)；每个物理步都重新调用控制器，[执行循环](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L300)。

证据：`dt=0.0005` 与 `0.00025` 对应 2 kHz、4 kHz 控制，位置增益分别为 **−45.5535**、**−51.2395**；其他增益也变化。现有半步长跳跃结果可表明两个联合配置都能工作，但不能单独归因于物理积分收敛。

建议：显式分离 `physics_dt`、`control_dt` 和未来 `policy_dt`，用整数分频和零阶保持连接；LQR 按固定 `control_dt` 设计。第一组实验只减小物理步长，保持控制器和命令时序不变；第二组再研究控制频率。延迟、丢样和策略降频也应通过同一时钟接口实现，避免另写训练专用步进循环。

### 4. P2｜MuJoCo 数值复位的处理在 viewer 与 ROS 中不一致

类型：已复现的适配器故障处理缺陷。viewer 检查非有限状态/时间倒退并暂停，[viewer 检查](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/viewer.py#L228)；无头运行会结束失败回合，[评估检查](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L309)；ROS 每个回调连续推进多个子步，却没有同等检查，[ROS 步进](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/ros2/wheellegged_ros/wheellegged_ros/sim_node.py#L169)。

触发与证据：在真实模型运行到 `0.006 s` 后，显式注入 `qvel=1e30`。MuJoCo 发出 QVEL 警告并把时间复位到 `0.0005 s`；viewer 正确暂停，ROS 则在后续子步抛出 `ValueError: jump supervisor time must be finite and monotonic`。这是故障注入，**不表示当前名义任务自然出现数值爆炸**。

建议：三种入口共享步进结果和故障状态。发现数值复位、非法状态或规定的严重求解警告时停止该回合，保留原因和最后有效状态，禁止在旧控制器/旧参考上继续推进；由明确 reset 同时重置物理、控制器、事件和时钟。RL 包装器须把它作为失败终止，不能让 MuJoCo 内部复位被解释为一次正常转移。

### 5. P1｜当前台阶基准和几何指标尚不能直接扩展成随机楼梯任务

类型：公开的基线边界，加一个已复现的扩展障碍。当前只有单块 **6 mm** 台阶和单边桥，[场景定义](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/model.py#L35)。台阶命令预先伸腿到 `−0.30 rad`，按固定时间以 `0.18 m/s` 接近；没有障碍检测或越障决策，[固定命令表](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L52)。`seed` 只记录索引，没有随机采样，[运行约定](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L274)。这些是诚实的回归基线设计，不应改写为“已具备通用楼梯控制”。

指标扩展障碍：轮底到地面的高度、平台/桥的几何判定读取全局 `TERRAINS`，而非该回合实际模型，[地面高度](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L76)、[平台验收](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L205)。将编译后的台阶改成 12 mm 的随机化原型后，几何查询仍返回 6 mm。原来的固定场景没有因此变错；但是仅随机修改 MuJoCo geom 会让模型与评价脱节。

建议：用每回合不可变的 `TerrainSpec` 同时生成碰撞几何、路径/落脚区域和评价参数，记录实际构造值；检查实体地形 geom 的接触归属。先扫描高度/轮半径比、台阶位置、宽度、接近速度和初始姿态，区分碰壳、打滑、力矩不足、姿态失稳和策略时机问题，再加入多级楼梯与训练/测试分离。不能通过修改阈值把真实失败变成成功，也不能把更换 seed 当作独立随机试验。

### 6. P1｜当前控制器读取仿真真值，不能直接成为可部署策略的观测接口

类型：已公开的建模假设，需要在学习前明确权限边界。[observe](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/state.py#L28) 直接读取 chassis 姿态、世界轮轴位置和刚体速度；控制器直接使用它及精确接触压力，[控制观测](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/control.py#L81)。ROS 发布的 IMU 并未反过来驱动这个控制器，当前 metadata 也明确标注了真值来源，[已有声明](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/evaluate.py#L347)。

影响：即使在 ROS 的 IMU 主题上加噪声，也不会自动改变平衡环所见的状态；若学习策略直接接收这些字段，会把不可直接测得的轮轴绝对位置、速度或精确接触力当作部署输入。

建议：保留 `TruthState` 用于评价和训练 critic，新增独立的 `PolicyObservation` 契约，限定 actor 可见的编码器、陀螺仪、经确认的姿态/速度估计、历史命令与历史动作。**只限制高层 actor 还不够**：只要底层 `BalanceController` 仍直接调用真值 `observe`，整个闭环依旧依赖真值。须向底层控制器注入统一的状态估计接口，再让 CLI、ROS 与学习环境共用；初期可明确命名为 oracle 实验，然后做传感器可用观测对照。噪声、偏置、延迟必须进入真实控制输入路径。先完成这个分离，再讨论加速度计改进，避免把额外传感器仅加在显示层。

另一个研究契约问题是控制器具有记忆：位置/偏航/速度参考、共同/差分腿参考、roll 积分、跳跃阶段及阶段计时都会影响下一步输出。同一机器人瞬时姿态和同一高层动作，在不同内部参考下未必得到相同响应。下一阶段应定义只读 `ControllerState`，向 actor/critic 提供部署时可获得的控制器状态，或明确采用历史窗口/循环策略并承认部分可观测性；不能把没有这些记忆的瞬时观测误称为完整 Markov 状态。尤其要防止策略通过积累隐藏的位置误差获得训练中的暂时收益。这是环境定义问题，不是把 LQR 的积分记忆本身判为缺陷。

### 7. P2｜执行器限幅已正确落实，但其动力学仍需辨识后才能支撑实机结论

类型：已有充分披露的工程代理，不是把缺少数据冒充代码 bug。当前轮驱动为电压/转速仿射模型加力矩上限，髋为限矩位置伺服，[模型映射](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/src/wheellegged/model.py#L105)。配置明确指出轮电压转矩系数、舵机内部刚度/阻尼、供电轨、实时电流限制和附件质量分配尚未辨识，[参数状态](https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim/blob/1cecec9f83af218b2b19618ab71cff5d529c2f99/config/hardware_parameters.json#L42)。

影响：测试证明输入/力矩遵守当前代理约束，不证明它重现了实际电流、带宽、死区、延迟、压降和连续热工况。学习算法可能利用这些代理中响应过快或过于理想的部分；换算法不能消除物理参数不确定性。

建议：先冻结名义 profile，再按参数来源设计敏感性试验；获得实测后建立版本化标定集。先比较固定控制器在不同参数下的表现，再比较重新整定后的结果，避免每个模型都单独优化而掩盖鲁棒性不足。不要为越台阶临时提高到堵转转矩，也不要把随机化候选范围标成测量置信区间。

## 最小复现和证据边界

可复现探针位于 [scripts/review_checks.py](../scripts/review_checks.py)，本次含 ROS 的小型结果保存在 [code_review_checks.json](evidence/code_review_checks.json)，包含脚本和实际运行源码 SHA-256。在已安装本项目依赖的 Python 环境中运行：

```bash
python scripts/review_checks.py
# 仅在兼容 ROS 2 的 Python 环境中启用这一项：
python scripts/review_checks.py --ros
```

默认不导入 ROS；`--ros` 才加入 ROS 源码路径并运行适配器回调故障探针，该项不是 DDS 集成测试。`--output` 可指定 JSON 路径，默认 `outputs/review-repro/review_checks.json`；运行期间临时切换到输出目录，MuJoCo 故障警告日志留在那里，完成后恢复调用者工作目录。该本地输出目录属于运行产物，不是公开证据链接。

报告使用 `status=observations_recorded` 表示探针已记录，**不是任务 PASS**；已知缺陷、未运行项与注入故障按实际结果保存，不用总括断言把预期故障解释成验收失败。探针没有改生产模型文件，没有把注入状态当成自由基座运动实验，也不增加或替代现有 140 项测试。

本审查没有发现足以否定既有五个名义任务、真实腾空证据或左右腿实测动作的依据。台阶失败边界需以专门的自由基座试验扫描给出，不能只从静态代码猜测最大高度。

## 下一阶段的模块和验收顺序

保留已有 MJCF、质量映射、控制器及其回归证据，先抽出共同契约，不做一次性重写：

| 模块 | 清晰职责 | 验收要求 |
| --- | --- | --- |
| `simulation.py` | 唯一物理步进与 reset；整数分频；返回状态、执行器输出、事件和 fault | CLI、viewer、ROS 和未来环境相同输入产生一致轨迹；数值复位必定终止回合 |
| `terrain.py` | 不可变 `TerrainSpec`；场景构造及实际几何/geom ID | 多回合、多高度相互隔离；几何和指标使用同一描述；边缘与墙面接触不冒充平台支撑 |
| `observations.py` | 真值、可测输入、估计状态和只读控制器状态分别定义；噪声/延迟进入高低层输入链 | actor 输入清单可审计；底层不再隐式读真值；控制器记忆可见或有明确历史建模 |
| `telemetry.py` | 版本化记录完整命令、参考、动作、实际力矩、接触和终止原因 | 左右请求可区分；回放完整；训练/评估同一 schema |
| `envs/wheellegged.py` | 薄环境适配层；动作归一化、奖励、结束条件、回合随机化 | 零残差严格复现冻结基线；动作不会写 `qpos/qvel`；无效数值不会作为有效训练样本 |

建议顺序：

1. **先建立失败地图。** 固定执行器约束，扫描台阶尺寸和接近条件，保存成功与失败的完整轨迹和接触证据；为人工停车/受阻参考管理补物理试验。
2. **补齐共同运行内核与日志。** 修复左右请求日志、ROS 故障一致性，分离时钟，重跑原 140 项回归并加入新的有针对性的验收。先做到物理步长收敛和多入口一致，再开始批量训练。
3. **先学有限残差/高层时机。** 第一版高层动作选择左右腿目标与接近速度，保留轮部 LQR 和跳跃安全监督器；动作仍通过实际位置舵机和轮电压接口。零残差必须退化为原控制器。具体算法选择由独立文献与实现比较决定，不在本审查中把 PPO/SAC 名字当作改进证据。
4. **再验证学习收益。** 同预算比较冻结 LQR、规则越障和学习残差；使用独立训练/测试地形与真实随机种子，报告成功率区间、姿态/闭链误差、碰壳、滑移、动作饱和、延迟及参数扰动敏感性。真值策略和可部署观测策略分开报告。
5. **最后推进实机迁移。** 用辨识结果替换工程代理，再验证实际观测与时序。加速度计优化属于这一清晰链路的一部分，不能用来掩盖场景定义、指标或执行器模型的问题。

这一路线的近期目标是解释并扩大可重复的台阶成功域，同时保持现有平衡、移动和跳跃能力；是否达到论文贡献要求，应由新方法、对照实验和泛化结果决定。
