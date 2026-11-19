# 开发与上下文记录

用户目标：基于世博微型轮腿车 CAD 与硬件代码，做可复现 ROS 2 + MuJoCo 仿真，覆盖平衡、前后移动、单边桥、台阶、跳跃；本地验证之后上传 GitHub。额外加速度计优化留到后阶段。当前工作分支尚未发布。

## 2026-11-20 WASD追加需求

- 用户明确选择“按住运动、松开自动停车”。原被动MuJoCo窗口无公开release回调，交互入口已改成自有GLFW窗口＋MuJoCo公开渲染，保留ROS使用的被动窗口生命周期helper。
- W/S为±.12m/s、A/D为±.4rad/s，支持组合与相反键抵消；release清对应目标，失焦/停车/暂停清按键，J仅按下沿发一次请求。没有全局键盘钩子、自动重复计时推断松键或瞬时修改物理速度。
- 51项真实窗口/程序事件注入检查通过，W+A/W+D各10项实际闭环检查通过，全套pytest更新为100 passed。新证据仍在viewer_smoke.json，之前16项旧按键探针已由Git历史保存。
- 纯MuJoCo帧缓冲截图`docs/media/wasd_viewer.png`已目视检查。五任务控制核心未变，不用把可视化重构当作新的动力学调参。
- GitHub连接器读取可用但写入403，本机无已配置Git认证；已改用用户登录的GitHub网页，按保留目录结构上传91+个文件。发布最后核对以远端逐文件内容hash为准。

## 2026-11-20 本地交付检查点

- 新hardware_2s五任务、五组额外扰动/半步长对照全部通过；正式结果`outputs/validated-hardware-v1`，41秒视频合集`demo.mp4`；小型报告归档`docs/evidence/hardware_v1`。
- 全套90项测试通过（另两条既有Lark弃用警告），依赖检查无冲突。BRep质量重聚合、独立COM弹道、速度坐标/Jacobian核验均完成。
- 原生viewer实现了手动控制、1–5自动演示、暂停、重置、相机与实时曲线；真实窗口16项代码回调注入集成检查通过。ROS DDS、colcon/无头launch和viewer外部指令联动均通过；窗口退出竞争已修，检查了子进程正常退出。
- 补充审阅4个固定版本开源项目，差异和未复制源码说明在`additional_projects.md`；官方8个MuJoCo skills已安装。
- 新模型估计质量317.649381g，默认轮±3.7V、髋±0.14709975Nm；微跳双轮净空6.985mm、COM离地后上升2.878mm。半步长COM上升有6.8%差异，实机增益/响应等尚待标定。
- 已按授权创建公开仓库 https://github.com/Wang-Changyu622/shibo-wheellegged-mujoco-sim ，中文标题保留用户指定文本。现在上传本地已验证版本并核对远端；不会上传原始私人截图或第三方CAD缓存。
- 额外加速度计优化仍未开始。后续研究应先完成实机测量、对照消融和不确定性范围，不能把当前基线称为RAL论文已完成。

## 2026-11-20 新硬件与可视化检查点

- 用户确认电池为2S标称7.4V，提供YT2208（41g）与STS3032（20.6g、额定1.5kgf·cm@6V）截图。默认profile改为`hardware_2s`，旧`reference`可显式复现。
- 参数推导和待测项见`hardware_parameters.md`；默认轮命令±3.7V（SinePWM），Ke与Uq配套换算，舵机额定限矩0.14709975Nm。电压-转矩增益仍待实测，不能声称硬件数字孪生已标定。
- 正在从STEP BRep逐部件修正质量与惯量，保留未知部件估计及转子/定子质量比例先验；不对全车强制重新归一到336g。
- 原reference v2五任务通过、58项测试通过及ROS DDS/build/launch证据已归档`docs/evidence/reference_v2`、`outputs/validated-v2`。这些不是新硬件验收结果。
- 单改执行器的探索性结果：平衡/移动/台阶/跳跃通过，单边桥连续路径检查未通过，保存`outputs/hardware_actuator_probe`，后续在新质量上重验和调节。
- 新发现：MuJoCo `mjOBJ_BODY`取惯性坐标系/质心，`mjOBJ_XBODY`取机体原点/坐标系。已修控制轮轴速度定义，ROS同步修正，新增独立位置微分与旋转坐标检查。
- 用户明确要求可交互可视化；正在实现固定物理步长、60Hz同步、键盘操作、1–5演示切换、暂停重置和实时状态。额外项目差异调查将写入`additional_projects.md`。
- GitHub已登录；用户指定中文名“世博轮足平衡车在mujoco中仿真实现”。README采用原名，GitHub合法slug为`shibo-wheellegged-mujoco-sim`，公开仓库创建表单已填写，按用户要求等待新默认配置本地验证后创建发布。
- 当前分支`codex/ros2-mujoco-baseline`，初始提交`a2f7746`；工作区后续修改仍在进行。收尾需要新版质量/参数/控制五任务、ROS与交互窗口验收，更新正式报告和文档，再提交发布。

下列历史记录保留用于理解设计过程；当前状态以上方检查点和正式验收证据为准。

## 已建立的基础

- 本地原始源在桌面 `Micro-Wheeled_leg-Robot`。STEP 与公开模型对应同一装配，换行差异已核验。图片两仓库和固件差异见 `reference_review.md`。
- 真实 CAD 网格 + 每腿四杆闭环 + 自由基座；无锁机身、位姿驱动或外部升力。第三方 36 MB 资产不纳入新仓库再分发，`scripts/fetch_assets.py` 固定提交逐文件校验恢复。
- 四执行器为两轮电压、两腿位置命令。轮半径18.5 mm，轮距87.3 mm，参考质量0.336 kg（非本用户实测），轮±7.4 V、髋±0.15 Nm。真实硬件仍待标定。
- 根据干涉扫描，把运行时髋命令收紧到[-.30,0] rad；原始资产保持不改。接触margin每表面50 µm。
- 控制独立模块：`model.py`、`state.py`、`lqr.py`、`control.py`、`evaluate.py`、`render.py`；ROS是薄适配，控制不依赖消息往返。
- LQR在CAD质量矩阵上做滚动约束投影，含轮电压增益/反电动势/关节阻尼，按实际腿角插值七组增益。腿差分控制roll，yaw差分控制轮电压。
- 已安装8个Google DeepMind官方MuJoCo skills，来源与hash见`mujoco_skill_status.md`。
- 本机已有RoboStack ROS2环境，项目.venv继承该环境；新机器依赖安装入口为`scripts/setup.sh`，ROS另需其环境。

## 已发现并修正的问题

1. 原模型+无控制会跌倒；不能拿静态预览冒充平衡。
2. 参考固件电压增益和大机器人力矩VMC不能直接互用。当前实现是电压LQR + 有界位置舵机协调。
3. 世界角速度不等于Euler角速度，已修正非零yaw的roll/pitch/yaw导数并加入数学回归。
4. 1 ms步长存在明显接触高频抖动；0.5/0.25 ms对照稳定。默认物理与控制均2 kHz，ROS发布100 Hz。
5. 第一次严格测试：平衡、前后、跳跃通过；台阶/桥8秒停车超速门槛，台阶下腿接触顶缘。失败保存在`outputs/baseline-v1`和`outputs/evaluation_development`。
6. 不删除腿碰撞。姿态扫描表明上阶前伸腿至-.30 rad可消除非轮支撑；命令计划公开记录。位置Q权重25→100使停车响应更快。
7. 跳跃早期计时落地切换晚约84ms，正在改为持续接触确认；必须区分轮净空约5–6mm和COM实际飞行上升约2.5mm。

## 收尾检查清单（以最终验收报告为准）

- 完成接触驱动跳跃监督器，冻结控制代码。
- 五任务严格验收、正负扰动、0.5/0.25 ms收敛、ROS DDS/build/launch刷新。
- 保存最终代码hash、每步轨迹NPZ、JSON指标、视频和图；旧失败不覆盖。
- `pytest`、依赖检查和独立动力学审查，完整README/限制/科研方案。
- 本地Git提交，收到目标仓库/可见性后处理GitHub发布。用户已被异步询问GitHub目的地与实际地形尺寸。

本记录是工作检查点，不是结果通过声明。正式最终结果以`docs/VALIDATION.md`为准。
