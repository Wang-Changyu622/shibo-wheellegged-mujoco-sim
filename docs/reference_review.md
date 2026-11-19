# 参考仓库与原硬件审计

核查日期：2026-11-20（Asia/Shanghai）。以下是来源审计及设计建议，不是本项目仿真通过报告。源代码行为优先于上游 README 的简述。本文未复制参考控制器实现。

## 固定来源

| 来源 | 本次检查的提交 | 用途 |
|---|---|---|
| [MuShibo/Micro-Wheeled_leg-Robot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot/tree/19a012ec8ee8b4f981aab887409a64b8dff37725) | `19a012ec8ee8b4f981aab887409a64b8dff37725` | 原机械 STEP、ESP32/SimpleFOC/STS3032 固件 |
| [dycsrl/Micro-Wheeled_leg-Robot-Model](https://github.com/dycsrl/Micro-Wheeled_leg-Robot-Model/tree/a330459b2d3e4567974bb6415abb51086fa5d683) | `a330459b2d3e4567974bb6415abb51086fa5d683` | STEP 派生网格、闭环 MJCF、几何与承重验证依据 |
| [Edmounds/wheel_legged_robot_sim](https://github.com/Edmounds/wheel_legged_robot_sim/tree/a0362990af34e723a75287765e48c586e9f10160) | `a0362990af34e723a75287765e48c586e9f10160` | LQR、高度控制、跳跃相位与试验架构参考 |

本地原工程 `/home/wcy/桌面/Micro-Wheeled_leg-Robot` 与 MuShibo 上述提交一致，本次读取时工作区干净。`1.RobotModel/OriginalRobotModel.stp` 的 SHA-256：

```text
4715cf940718af95872bd2a771fdde61de3376bb748936e2299f94fa76416947
```

该 SHA 与本次下载的上游 STEP 一致。复现应固定提交和资产哈希，不能只记录 `main` 或 `master`。

## 机械及执行器差异

| 项目 | MuShibo / dycsrl 微型机器人 | Edmounds 参考机器人 | 对本项目的意义 |
|---|---|---|---|
| 机构 | 每侧单主动髋 + 被动四杆闭链 + 独立驱动轮 | 每侧单主动腿电机 + 四杆闭链 + 独立驱动轮 | 自由度相似，几何/Jacobian/惯量不能共用 |
| 轮半径 | CAD 18.5 mm | 36 mm | 轮角速度到线速度、力矩到推力均需重新换算 |
| 质量 | dycsrl 复刻实测总质量 0.336 kg，电池 0.058 kg | 参数注释记录重配后 2.20 kg | 0.336 kg 不是当前用户实物称量；不可写成已标定 |
| 腿执行器 | STS3032 位置舵机，内部动态未知 | 仿真力矩执行器 | 不能直接注入参考 VMC 的 N·m 指令 |
| 轮执行器 | SimpleFOC 电压转矩模式 | 仿真力矩执行器 | 控制量 V 与 N·m 必须明确分开 |
| 坐标 | dycsrl 转换后沿 X 前后、Y 左右、Z 向上 | 参考模型沿 +Y 前进，轮轴 +X | 姿态与轮方向符号需由当前 MJCF 验证 |
| 腿工作区 | dycsrl ctrl ±0.3 rad、关节 ±0.35 rad；部分正角存在干涉 | 高度 LUT 约 0.0784–0.1537 m | 不能按参考高度或 XML 范围宣称机械安全 |

dycsrl 将 157 个 STEP 实体中的原电池移除，保留 156 个真实零件网格；电池采用尺寸 65×26.5×15 mm 的盒体替代。非电池质量按材料类别/体积归一化，电池中心为假设值，动力学仍需实测辨识。其四杆使用 site-connect equality 闭合，不能用删除闭环约束来简化通过率。

来源：[建模假设](https://github.com/dycsrl/Micro-Wheeled_leg-Robot-Model/blob/a330459b2d3e4567974bb6415abb51086fa5d683/docs/model-assumptions.md)、[模型语义](https://github.com/Edmounds/wheel_legged_robot_sim/blob/a0362990af34e723a75287765e48c586e9f10160/src/model_semantics.py)。

## 原硬件代码的实际控制逻辑

源文件：[wl_pro_robot.ino](https://github.com/MuShibo/Micro-Wheeled_leg-Robot/blob/19a012ec8ee8b4f981aab887409a64b8dff37725/3.Software/wl_pro_robot/wl_pro_robot.ino)。

1. 平衡是四项状态反馈，源码称 LQR，但仓库该文件没有 A/B 矩阵、Q/R 权重或 Riccati 求解过程。`pitch`/`gyroY` 是度/度每秒，轮位移/速度是左右轴角/角速度的负平均（rad、rad/s）。默认四项 P 系数是 `1, 0.06, 0.5, 0.7`；速度系数随高度从 0.7 降到 0.6/0.5。初始 pitch 零点 −2.25°。
2. 前后控制采用 `0.1 * lowpass(joy_y)` 的轮角速度参考；有运动指令、停车、快速推动时重置位移参考。还有小输出积分补偿和重心零点慢调整，不能简单视为一组固定线性 K。
3. 左右轮目标为 `−0.5*(balance ± yaw)`。`TorqueControlType::voltage`、`MotionControlType::torque` 的命令单位是电压；固件供电设置为 8 V。这里不是电流闭环或精确轮力矩控制。
4. 腿位置：左 `2048+12+8.4*(height−32)−roll_correction`，右 `2048−12−8.4*(height−32)−roll_correction`。常态分别限制为 `[2110,2510]`、`[1586,1986]` counts；常态舵机 `Speed=200, ACC=8`。舵机控制为 1 Mbps 串行同步写，不能从固件零位直接推导 CAD 零位。
5. roll 采用低通后 P 控制，默认 `P=8 counts/deg`，IMU X 轴角加 2° 偏置。两腿电气计数同向改变量对应几何差动，仿真要按各关节轴重新确定符号。
6. 跳跃由 `dir:jump→stop` 边沿触发：先 height=80 指令，再在循环计数 30 左右收至 height=40，循环计数 200 后结束。`Speed=0, ACC=0` 表示使用该舵机协议的特殊速度/加速度设置，不能当成物理零速度。原循环无固定周期，不能把 30/200 直接解释为毫秒。
7. 原离地判据使用轮速差/轮速阈值和跳跃标志；这不是接触传感器，轮速差也未除以 dt。仿真基线可使用明确标注的真值接触；后续实机状态估计必须单独验证。
8. |pitch|>25° 禁轮输出，扶正至 <10° 后经过循环计数恢复。仿真采用物理秒和锁存故障状态更易复现。

控制协议：[robot.cpp](https://github.com/MuShibo/Micro-Wheeled_leg-Robot/blob/19a012ec8ee8b4f981aab887409a64b8dff37725/3.Software/wl_pro_robot/robot.cpp) 的 WebSocket JSON 包含 `mode=basic`、`dir`、`height`、`roll`、`linear`、`angular`、`stable`、`joy_x`、`joy_y`。实际速度/转向主要读 joystick 字段；不能把存在于 JSON 的字段视为均已接入运动控制。20 字节 UART 转发函数存在但调用被注释，不能声称原固件已有 ROS 2 串口接口。

## Edmounds 可借鉴的结构与不可照搬项

- `balance_lqr.py` 采用降阶离散 LQR，状态 `[pitch,pitch_rate,roll,roll_rate,wheel_velocity]`，两路虚拟输入为轮共模和腿差模；位置控制放在慢外环。增益随腿高更新，平衡俯仰零点由 COM 相对轮轴几何计算。
- 高度模块通过闭链 LUT 将高度转成关节角，配合关节 PD、速度及动力学前馈。虽然命名 VMC，当前主要实现不是简单的笛卡尔虚拟弹簧 `JᵀF`。可以借鉴闭链运动学/工作区检查，不能把“VMC”标签等同于实机力矩能力。
- `STAND→CROUCH→EXTEND→FLIGHT→LAND` 明确各相位输入分配，避免平衡和起跳同时争用腿执行器。起跳和落地应由持续接触条件与速度确认，而不只看定时器。
- 当前 README 与代码存在差异：README 写 FLIGHT 轮腿置零，但 `vmc.py` 实际用对称腿 pitch-rate 阻尼，限幅达 ±3.5 N·m；`combined.py` 的站立腿 roll LQR 输出已置零，roll 由高度差调节。以固定提交的源码行为为准。
- `balance_lqr.py` 的 reduced model 固定 `dt=0.002`，而物理步长另设；迁移时应由实际控制采样周期生成 A/B，不能埋入固定值。模型把轮虚拟力矩分配给双轮时，也要检查总力矩/每轮力矩的一致性。
- 聚合 `contact_count` 不能证明两轮分别接地，特别是当前微型模型有选择性自碰撞。每侧必须按轮 geom 与 terrain geom 配对统计法向力/接触持续时间。
- 两套模型量级差异使 `kp=30 N·m/rad`、`kd=1`、空中 ±3.5 N·m、45 mm 起跳伸腿行程等不能照搬。参考没有为当前微型机器人证明台阶、单边桥或跳跃能力。

对应源码：[LQR](https://github.com/Edmounds/wheel_legged_robot_sim/blob/a0362990af34e723a75287765e48c586e9f10160/src/controllers/balance_lqr.py)、[组合控制器](https://github.com/Edmounds/wheel_legged_robot_sim/blob/a0362990af34e723a75287765e48c586e9f10160/src/controllers/combined.py)、[腿高度/VMC](https://github.com/Edmounds/wheel_legged_robot_sim/blob/a0362990af34e723a75287765e48c586e9f10160/src/controllers/vmc.py)、[相位机](https://github.com/Edmounds/wheel_legged_robot_sim/blob/a0362990af34e723a75287765e48c586e9f10160/src/controllers/phase.py)。

## 建议的本项目基线

平衡内环使用 SI 单位、明确采样周期的轮共模 LQR，配合限幅、慢速位移/速度参考和 yaw 差动；先验证单输入符号、接地线性化与闭环极点，再测试小角扰动。腿部保留位置舵机语义，通过闭链 LUT/逆运动学生成高度与左右差动目标，并显式限制目标速度、关节范围、舵机力矩。跳跃使用物理秒的相位机、逐轮接触判断和着地恢复；每个 phase 清晰列出启用反馈通道。ROS 2 负责命令/状态桥接，控制核心与 MuJoCo runner 保持可脱离 ROS 的确定性测试接口。

dycsrl 工程初值为舵机 `kp=6.0830245282658 N·m/rad`、`kv=0.13525675068501658 N·m·s/rad`、力矩限幅 ±0.15 N·m；轮近似 `τ=0.01V−0.0001ω`、电压 ±7.4 V、力矩 ±0.08 N·m。它们均未实机辨识。其 [承重报告](https://github.com/dycsrl/Micro-Wheeled_leg-Robot-Model/blob/a330459b2d3e4567974bb6415abb51086fa5d683/docs/SERVO-HOLD.md) 仅证明三姿态/同向承重，锁 roll 台架的差动目标失败；引用的额定 0.1471 N·m / 短时堵转 0.4413 N·m 不能当成任意时长可用扭矩。若仿真跳跃需要高于保守包络的模型，必须单列实验配置与假设，不能悄悄放宽限值再称原硬件成功。

## 试验与论文证据边界

“能播放动作”和“闭环任务成功”应分开。最低任务证据包括：

- 平衡：自由基座、扰动初值、足够持续时间、姿态 RMS/峰值、位置漂移、执行器饱和率。
- 前后移动：正负速度阶段、停止回稳、速度 RMSE、侧向漂移与 yaw 误差。
- 单边桥：明确桥长/宽/高度/边缘，记录左右轮分别在不同高度支撑，完整通过后回稳；不能用两轮同高平台代替。
- 台阶：报告高度/轮半径比，确认从低面到高面的实际 COM/轮位置迁移并回稳；原地抬腿不算上阶。
- 跳跃：双轮持续离地时间、轮底净空、COM 顶点、落地后连续稳定时间；姿态弹一下或清空接触参数不算。
- 数值可信性：闭环约束残差、穿透、警告、半步长复验、质量/摩擦/延迟/噪声扰动；保存失败与成功，避免只选好看的轨迹。

现阶段属于可重复仿真与传统控制基线。RAL 级别论文还需要明确的新贡献、对照与消融、多种初值和参数随机化、统计置信区间，以及与真实平台的动力学/任务验证。LQR+VMC 组合本身和已有开源演示不能独立构成创新性证明。额外加速度计优化属于后续研究，不应混入当前基线而影响归因。

## 来源与发布范围

Edmounds 仓库包含 Apache-2.0 LICENSE。MuShibo 固件源文件带完整 MIT 风格许可头；本次未发现覆盖整个 CAD/硬件工程的独立 LICENSE。dycsrl 也没有独立 LICENSE，仅有原作者权利保留及第三方来源说明。因此不可把整个项目及参考 CAD/网格统称为新项目的 MIT/Apache 内容。发布时保留来源、提交、资产哈希和已有声明；若第三方资产授权范围尚不清楚，可发布独立实现及下载/重建脚本，将第三方资产留在用户本机而不重新打包到新仓库。此处是仓库文件审计结论，不是法律意见。
