# 补充开源项目审阅与可视化设计

审阅日期：2026-11-20。除 [原始硬件、CAD 与首轮参考](reference_review.md) 外，又通过 GitHub 原仓库核验了下面四个项目。下文固定到具体提交，结论来自 README、模型、控制入口及可视化源码的静态审阅；没有运行这些项目，因此不把上游演示或自述成功率当作本项目的实测结果。没有下载额外模型依赖或复制其控制器代码。

## 固定版本与适用范围

| 项目 | 固定提交 | 模型及执行器 | 许可核验 | 本项目主要参考价值 |
| --- | --- | --- | --- | --- |
| [oyzlight/mujoco-lqr-vmc-wheel-legged-robot](https://github.com/oyzlight/mujoco-lqr-vmc-wheel-legged-robot/tree/74fb426bdb8d86d42f70adfbd98977dad4aea04f) | `74fb426bdb8d86d42f70adfbd98977dad4aea04f` | 自由基座；每侧两个闭环；4 个髋力矩电机＋2 个轮电机 | 该版本根目录未找到 LICENSE，GitHub 未识别许可；仅作资料审阅 | 原生 MuJoCo 键盘控制、显式重置、已知台阶辅助 |
| [CosmosMount/wbr_mujoco](https://github.com/CosmosMount/wbr_mujoco/tree/81a1035e66315a46abd1f0ca4c11ec40596b95c5) | `81a1035e66315a46abd1f0ca4c11ec40596b95c5`，`v2` 分支 | 自由基座；4 个 connect 闭链约束；4 个腿电机＋2 个轮电机 | 根目录未找到 LICENSE，GitHub 未识别许可；仓库已归档 | C++ 控制/仿真通信分层、分组在线曲线 |
| [Skythinker616/foc-wheel-legged-robot](https://github.com/Skythinker616/foc-wheel-legged-robot/tree/e2444395dd3a76c20b0683fbb1e123c21186a502) | `e2444395dd3a76c20b0683fbb1e123c21186a502` | 每侧双输入五杆闭链；4 个髋 BLDC＋2 个轮 BLDC | [GPL-3.0](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/LICENSE) | 几何、VMC、变腿长 LQR 与实机代码之间的对应关系 |
| [LYHrmer/wheel-legged-control-lab](https://github.com/LYHrmer/wheel-legged-control-lab/tree/a3a830a2008095057d74c413a8847829d6b113b5) | `a3a830a2008095057d74c413a8847829d6b113b5` | D1 四轮串联腿，16 个执行器；不是本车的双轮闭链结构 | [项目代码 MIT](https://github.com/LYHrmer/wheel-legged-control-lab/blob/a3a830a2008095057d74c413a8847829d6b113b5/LICENSE)；模型来源和独立许可另见其 model card | 交互 HUD、状态来源标注、场景重置与渲染质量分离 |

“公开可读”不等于具有明确的再分发许可。本次新增参考没有作为第三方源码并入本仓库；本仓库新增代码的 MIT 声明不覆盖这些上游项目。

## 两个直接相关的 MuJoCo 闭链项目

**oyzlight** 的 [robot.xml](https://github.com/oyzlight/mujoco-lqr-vmc-wheel-legged-robot/blob/74fb426bdb8d86d42f70adfbd98977dad4aea04f/robot.xml) 用 freejoint、每侧两个 site connect 和额外关节耦合约束构成机构；四髋电机限幅 ±7 N·m、两轮 ±2 N·m。它与本车的“每腿一个位置舵机＋一个轮电机”输入维数不同，不能直接移植 VMC 的双髋力矩输出或 LQR 增益。

其 [run_keyboard.py](https://github.com/oyzlight/mujoco-lqr-vmc-wheel-legged-robot/blob/74fb426bdb8d86d42f70adfbd98977dad4aea04f/run_keyboard.py) 把按键放进队列，由仿真循环处理，使用 `launch_passive`，相机只跟随水平位置。普通跳跃、已知台阶辅助、动力学自起身与 R 重置有不同入口。[README](https://github.com/oyzlight/mujoco-lqr-vmc-wheel-legged-robot/blob/74fb426bdb8d86d42f70adfbd98977dad4aea04f/README.md) 的 15 cm 台阶演示依赖固定几何和助跑，不是未知地形感知；文档也明确质量、惯量及力矩速度包络并非实测标定。因此适合参考交互组织，不能据此宣称本车也能越过同等台阶。

**CosmosMount** 的 [mjcf/mjmodel.xml](https://github.com/CosmosMount/wbr_mujoco/blob/81a1035e66315a46abd1f0ca4c11ec40596b95c5/mjcf/mjmodel.xml) 确认存在真实闭链 equality connect，腿/轮力矩上限分别为 ±20/±5.2 N·m。其 [README](https://github.com/CosmosMount/wbr_mujoco/blob/81a1035e66315a46abd1f0ca4c11ec40596b95c5/README.md) 使用 C++17、外部 `mujoco_interface` 和 eCAL，共享 YAML 配置，不是可以直接接入本项目的 ROS 2 节点。

最有用的是 [runtime/src/services.cpp](https://github.com/CosmosMount/wbr_mujoco/blob/81a1035e66315a46abd1f0ca4c11ec40596b95c5/runtime/src/services.cpp) 的浏览器遥测：高频控制与约 50 Hz 显示解耦，分别绘制姿态、目标/实际速度、LQR 输出、实际电机力矩、腿长及支撑力。由此得到本项目的设计建议：控制电压、位置舵机目标与实际关节力矩必须分别显示单位；总接触数不能代替左右轮各自的地面支撑状态。

## 硬件推导与交互可视化补充

**Skythinker616** 不是 MuJoCo 工程，其 [matlab/README.md](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/matlab/README.md) 采用 MATLAB/Simscape，并把推导结果生成给 ESP32 控制代码。[leg_func_calc.m](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/matlab/leg_func_calc.m) 明确以两髋角求五杆交点，再求雅可比和虚拟力到电机力矩的映射。这是检查几何闭环、虚功关系、变腿长增益来源的好参考；它的双髋力矩控制仍不等同于本车单舵机角度接口。这里只参考推导流程，没有复制 GPL 代码。

**LYHrmer** 的 [D1 model card](https://github.com/LYHrmer/wheel-legged-control-lab/blob/a3a830a2008095057d74c413a8847829d6b113b5/docs/d1_model_card.md) 说明模型约 48.15 kg、16 个执行器及上游模型来源。其 [交互课程文档](https://github.com/LYHrmer/wheel-legged-control-lab/blob/a3a830a2008095057d74c413a8847829d6b113b5/docs/interactive_course.md) 和 [键盘 viewer](https://github.com/LYHrmer/wheel-legged-control-lab/blob/a3a830a2008095057d74c413a8847829d6b113b5/scripts/d1_keyboard_viewer.py) 将按键状态、仿真时间和 HUD 渲染分开，低画质只降低视觉开销，场景切换明确执行重置。课程入口的跳跃能力不能混写为其当前 B22 主运行入口的能力，也不能当成本项目的动力学基准。

这次审阅还提示了一个实际坐标系陷阱：`mj_objectVelocity` 的 `mjOBJ_BODY` 使用惯性质心和惯性主轴，`mjOBJ_XBODY` 才使用普通 body 原点及轴。已通过本模型的非零惯性偏置独立复现，并用 [MuJoCo 官方实现](https://github.com/google-deepmind/mujoco/blob/3.3.7/src/engine/engine_core_util.c) 核验。ROS `base_link` twist 应对应普通 body 坐标；独立测试应由 body 原点雅可比乘 `qvel` 再旋转得到期望值，避免拿同一个接口自证正确。

## 本项目可视化应呈现的内容

以下是结合本车接口得出的实现建议，不是对上述项目功能的逐项复述：

1. 同一物理步长和控制器供无界面基准、交互 viewer 与 ROS 使用；显示刷新率降低时仍执行完整物理步。
2. HUD 明确显示硬件参数 profile、场景、仿真时间、实际实时率、手动/演示模式、目标及实际速度、pitch/roll、跳跃状态与左右轮支撑力。
3. 将轮电压 V、髋目标 rad 与实际执行器力矩 N·m 分开；显示限幅状态，不把控制输入当作测得的输出力矩。
4. 明确区分“停车”“暂停”“重置”。重置仅用于新回合，不作为控制恢复能力；场景切换可重置，但应让用户看见这一语义。
5. 接触点/接触力可切换显示，相机保留地面高度参考。播放单边桥、台阶、跳跃时同时显示轮接触与 phase，避免仅凭画面判断完成。
6. 仿真真值标注为真值；未来加入加速度计/估计器后，区分估计与真值，并用相同记录和指标比较。图形演示与成功率、扰动裕量、参数误差实验分别报告。

新增参考帮助完善可视化和接口审查，不替代本仓库的真实闭链、执行器限制、硬件质量映射及本地可复现实验。
