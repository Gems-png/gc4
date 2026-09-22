# 识别位于地面的黑色的同心圆形 (纯函数模块, 由 MainCam 通过 import 引入)
import cv2


def detect_circle_by_alt(self, frame):
    """检测一帧图像中的地面同心圆。

    通过传入节点实例 self, 可直接使用节点上下文:
        self.hsv          HSV 分割参数
        self.get_logger() 日志
        self.detect_material(frame)  复用 material 模块的物料检测
    TODO: 待补齐具体识别算法。
    """
    _ = frame
    self.get_logger().debug('detect_circle_by_alt: 待实现')
    return None
