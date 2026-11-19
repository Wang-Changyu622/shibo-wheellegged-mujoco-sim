# 官方 MuJoCo Agent Skills 安装记录

核查与安装日期：2026-11-20（Asia/Shanghai）。

已确认 Google DeepMind 的 MuJoCo 官方仓库提供 Agent Skills，并完成全部 8 项安装。未使用第三方仿冒或同名技能。

- 官方目录：[google-deepmind/mujoco/doc/skills](https://github.com/google-deepmind/mujoco/tree/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills)
- 官方说明：[MuJoCo Agent Skills README](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/README.md)
- 固定源码提交：`05172694c8d78cc18106ae580fcacdf9a1ebd5db`
- 安装器：`/home/wcy/.codex/skills/.system/skill-installer/scripts/install-skill-from-github.py`。
- 安装方法：一次下载该提交下的 8 个技能目录到临时目录，再按各文件的 YAML `name` 移入 Codex skills 目录；未修改上游技能文件。
- 自动技能目录将于下一轮对话可用；当前轮可直接读取绝对路径中的 `SKILL.md`。

## 已安装技能

| 技能 | 用途 | 本机位置 |
| --- | --- | --- |
| [mujoco-python](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/python/SKILL.md) | Python 仿真、状态、传感器与线性化 | `/home/wcy/.codex/skills/mujoco-python/SKILL.md` |
| [mujoco-spec-editing](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/spec_editing/SKILL.md) | MJCF/MjSpec 建模、关节、执行器与碰撞 | `/home/wcy/.codex/skills/mujoco-spec-editing/SKILL.md` |
| [mujoco-rendering](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/rendering/SKILL.md) | 离屏图像、深度、视频渲染 | `/home/wcy/.codex/skills/mujoco-rendering/SKILL.md` |
| [mujoco-studio](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/studio/SKILL.md) | 交互式查看器和 Studio | `/home/wcy/.codex/skills/mujoco-studio/SKILL.md` |
| [mujoco-gui](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/gui/SKILL.md) | ImGui/ImPlot 界面 | `/home/wcy/.codex/skills/mujoco-gui/SKILL.md` |
| [mujoco-accelerated](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/accelerated/SKILL.md) | MJX/MJWarp GPU 与批量仿真 | `/home/wcy/.codex/skills/mujoco-accelerated/SKILL.md` |
| [mujoco-parity](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/parity/SKILL.md) | CPU/GPU 引擎一致性检查 | `/home/wcy/.codex/skills/mujoco-parity/SKILL.md` |
| [mujoco-deformables](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/doc/skills/deformables/SKILL.md) | 柔性体建模 | `/home/wcy/.codex/skills/mujoco-deformables/SKILL.md` |

## 适用于本项目的使用约束

本项目优先读取 `mujoco-python`、`mujoco-spec-editing` 和 `mujoco-rendering`。技能来自官方 main 分支，可能包含尚未进入已安装 MuJoCo wheel 的新 API；应固定实际运行依赖版本，并通过本地编译、运行与版本检查验证 API。技能安装不等于安装 MuJoCo 仿真运行库。

保留上游文件原文后，技能中的相对链接仍按 MuJoCo 源码树解释。独立安装时应使用技能列出的官方在线手册回退，或本地 wheel 若附带的 `mujoco/doc`。官方文档：

- [稳定版 Python 接口](https://mujoco.readthedocs.io/en/stable/python.html)
- [稳定版 MJCF 建模](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [稳定版 MJCF XML 参考](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [官方 LQR 示例](https://github.com/google-deepmind/mujoco/blob/05172694c8d78cc18106ae580fcacdf9a1ebd5db/python/LQR.ipynb)

## 文件校验

以下为安装后的 `SKILL.md` SHA-256：

| 技能 | SHA-256 |
| --- | --- |
| `mujoco-python` | `4d1a97ea9f71ab12795036fd0784c6e400407e76b85cc53d76b3d386fc6e9bc6` |
| `mujoco-spec-editing` | `3b3f8d3e9f41e3d5b9166ad88963e5c6c841b802742d27244ac2d1bc0ba9abe5` |
| `mujoco-rendering` | `5238504466a752135f51e3c9420618f646f1205326e3327bc85274c73fe628b3` |
| `mujoco-studio` | `cf43bd56f570659e92b7e8206510792275328d970e08946cbc57fcf5f03e63aa` |
| `mujoco-gui` | `34c65c99439cf705fe43226c41feb0c04a6bdbc4ff9613ab17c48afbdedc7c8f` |
| `mujoco-accelerated` | `8378d89270e6b4794bf5d2aa7ccfb53b28822b4b26d29a02c2e0316cf1fa1bd0` |
| `mujoco-parity` | `ad37929c0a5819153ff9546ae7bd5e182852e2c0f25f88ea54e82c6cc6dd0538` |
| `mujoco-deformables` | `7494f44d514048300db17f63c56a0d5ade12716ea6f3bbe54f3c44028fd6c46c` |
