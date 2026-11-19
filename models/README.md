# 模型入口

本项目在 Python 入口 `wheellegged.model.load_model(terrain)` 中组合场景，避免为三个地形复制整份 CAD XML。源资产由安装脚本下载到 `assets/cad/`，保持来源文件不变；运行时增加地形和命名传感器。

```python
from wheellegged.model import load_model, make_data

model = load_model("flat")  # flat / bridge / step
data = make_data(model, hip_angle=-0.20)
```

若需要独立 XML，可在项目根目录导出本机绝对网格路径：

```bash
PYTHONPATH=src .venv/bin/python -c 'from wheellegged.model import model_xml; from pathlib import Path; Path("models/flat.generated.xml").write_text(model_xml("flat"))'
```

`*.generated.xml` 包含本机路径，应重新生成，不应提交。参数来源及模型边界见 `docs/model_provenance.md`。
