# 第三方来源

本项目的 MIT 许可只覆盖本项目新增实现，不改变上游内容的权利和条件。

| 内容 | 来源 | 本仓库处理 |
|---|---|---|
| 原始机械 CAD、硬件、固件 | [MuShibo/Micro-Wheeled_leg-Robot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot) | 参考原始设计和控制约定，未把原工程整体重新授权 |
| CAD 网格、闭环 MJCF、资产清单 | [dycsrl/Micro-Wheeled_leg-Robot-Model](https://github.com/dycsrl/Micro-Wheeled_leg-Robot-Model) | 下载脚本固定提交并逐文件校验；资产缓存不随新仓库再分发 |
| LQR/VMC 分层架构参考 | [Edmounds/wheel_legged_robot_sim](https://github.com/Edmounds/wheel_legged_robot_sim) | 独立实现微型车控制；未复制其大机器人控制增益或力矩接口 |
| MuJoCo 及官方 agent skills | [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) | 通过正式 Python 包运行；skills 安装在用户 Codex 目录，未打包进此仓库 |

原始工程贡献者包括穆世博（Mu Shibo / MuShibo）与李育锋（Li Yufeng）；CAD→MJCF 整理由 dycsrl 完成。模型预览与实验画面使用该来源的机械外观，应连同此来源说明保留。

上游公开不等于所有资产都具有同一通用许可证。当前未发现涵盖原 CAD/硬件和 dycsrl 资产的独立通用 LICENSE，因此本项目采用用户本机下载与固定哈希恢复方式，不将它们纳入本项目 MIT。固定版本、文件范围、原始声明与详细审计见 `scripts/asset-lock.json` 和 `docs/reference_review.md`。

Python/ROS 依赖各自保留其许可证。所有实际版本由验证报告记录，本项目没有修改依赖包的授权条件。
