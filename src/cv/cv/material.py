"""
通用物料识别算法库 — 台组合体（界面可能是圆形，也可能是三角形，展示不确定）, 无内参依赖，直接从图像中提取几何量。

由 MainCam 通过 `from cv import material` 引入, 经 self.detect_material() 调用。
原 CLI / 摄像头交互 / 合成图像自检部分已删除。
"""

