"""物料识别程序 — 圆台组合体, 无内参依赖

物料几何 (回转体, 单位 mm):
    上圆柱   r=15, h=8            z ∈ [52, 60]
    圆台     r: 25→15, h=44       z ∈ [18, 52]
    下圆柱   r=25, h=8            z ∈ [0, 18]
    总高 60, 底径 50, 顶径 30

场景约定:
    - 物料正立于平面上 (小端在上, 大端在下), 相机可从"平视 → 略俯视 → 正俯视"任意角度看。
    - 相机内参未知; 只有一个可选的"单点距离标定"用于绝对深度。

核心算法 (不需要内参):
    1. HSV 颜色分割 → 二值掩码
    2. **中央连续域优先**: 用 connectedComponents 保留位于画面中央的最大连通斑块,
       抑制画面边缘的相似色干扰 (纹理墙皮、灯光、其他物体)
    3. 掩码 minAreaRect → 剪影长短边、中心、粗略轴向
    4. 依剪影长/短边比自动分派:
         - 比值 ≥ 1.10 → side_view: 沿轴向切片给底径投影、找顶面平台反算 pitch
         - 比值 < 1.10 → top_down: 只看剪影短轴, 视为正俯视, pitch=90°
       (60/50 ≈ 1.20; 平视时 ≈ 1.20, 40° 时 aspect 达到峰值 ~1.44,
        70° 起持续下降, 90° 时 = 1.0. 用 1.10 阈值切换是安全的.)
    5. 距离: D = K / max_width_px, K = 单点标定常数, 存 depth_calib.json.
       用 minAreaRect 短边替代 bottom_slab_width, 两种视角下都恒等于底径投影.

输出 (MaterialReading):
    - color: 颜色标签
    - view_mode: "side" 或 "top_down"
    - center_px: 像素中心 (u, v)
    - axis_angle_deg: 物料轴在图像内相对竖直方向的旋转角 (deg, top_down 时为 0)
    - pitch_deg: 相机相对物料水平面的俯仰 (0=平视, 90=正俯视, side_view 无法估计时 None)
    - distance_mm: 相机到物料中心距离 (None 表示没标定)
    - max_width_px / top_ellipse / bottom_width_px: 原始几何量
    - confidence: 0-1

用法:
    python material_detector.py --test                 合成图像自检
    python material_detector.py --image path.png       处理单张图片
    python material_detector.py --camera 1             实时摄像头
    python material_detector.py --camera 1 \
        --calibrate 300 --color red                    以 300mm 距离标定, 保存 K

按键 (摄像头模式):
    q  退出
    c  切换目标颜色 (auto→red→yellow→…)
    s  保存当前帧和读数
    k  以当前帧对当前距离做即时标定 (需要 --calibrate)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


# ============================================================
# 物料几何
# ============================================================

@dataclass
class MaterialSpec:
    """圆柱+圆台+圆柱, 全部 mm"""
    bottom_cyl_h: float = 8.0
    bottom_r: float = 25.0
    frustum_h: float = 44.0
    top_r: float = 15.0
    top_cyl_h: float = 8.0

    @property
    def total_h(self) -> float:
        return self.bottom_cyl_h + self.frustum_h + self.top_cyl_h

    @property
    def bottom_d(self) -> float:
        return 2 * self.bottom_r

    @property
    def top_d(self) -> float:
        return 2 * self.top_r

    def radius_at(self, z: float) -> float:
        """z ∈ [0, total_h] 处的物料外半径"""
        if z <= self.bottom_cyl_h:
            return self.bottom_r
        if z >= self.bottom_cyl_h + self.frustum_h:
            return self.top_r
        t = (z - self.bottom_cyl_h) / self.frustum_h
        return self.bottom_r + t * (self.top_r - self.bottom_r)


# ============================================================
# 颜色分割
# ============================================================

HSV_RANGES: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
    "red":        [((0, 110, 90),   (10, 255, 255)),
                   ((170, 110, 90), (180, 255, 255))],
    "yellow":     [((20, 110, 110), (33, 255, 255))],
    "green":      [((38, 70, 60),   (85, 255, 255))],
    "blue":       [((100, 120, 80), (128, 255, 255))],
    "light_blue": [((86, 60, 130),  (100, 255, 255))],
    "black":      [((0, 0, 0),      (180, 90, 45))],
}
_CHROMATIC = ["red", "yellow", "green", "blue", "light_blue"]
_MIN_MASK_PIXELS = 400
_MIN_COMPONENT_PIXELS = 300      # 单个连通域至少这么大才考虑

_MORPH = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
_MORPH_SMALL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def _mask_of_color(hsv: np.ndarray, color: str) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    # 先 open 掉小点, 再 close 补洞. open 用 3x3 更温和, 避免吃掉薄剪影
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_SMALL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH)
    return mask


def _keep_central_component(mask: np.ndarray) -> Tuple[np.ndarray, int]:
    """从二值掩码中挑出"最靠中央的大连通域", 抑制画面边缘的相似色干扰。

    评分 = 像素数 × (1 + 3 × centrality), centrality ∈ [0, 1] 表示与画面中心的接近度。
    这样即使边缘有稍大的斑块, 只要中央有个中等大小的连续物料掩码, 就会被选出来。
    对底部一般都占画面主体的物料很稳; 若中央完全空白, 退化为选最大连通域。

    Returns: (只保留中央域的 uint8 mask, 该域像素数)
    """
    if cv2.countNonZero(mask) == 0:
        return mask, 0
    h, w = mask.shape[:2]
    cx0, cy0 = w * 0.5, h * 0.5
    diag = math.hypot(w, h) * 0.5   # 归一化用的最大距离

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best_i, best_score, best_area = -1, -1.0, 0
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < _MIN_COMPONENT_PIXELS:
            continue
        ccx, ccy = float(centroids[i, 0]), float(centroids[i, 1])
        dist = math.hypot(ccx - cx0, ccy - cy0)
        centrality = max(0.0, 1.0 - dist / diag)     # 1=画面中心, 0=四角
        score = area * (1.0 + 3.0 * centrality)
        if score > best_score:
            best_score, best_i, best_area = score, i, area

    if best_i < 0:                                    # 没有够大的组件 → 直接放弃
        return np.zeros_like(mask), 0
    cleaned = np.where(labels == best_i, 255, 0).astype(np.uint8)
    return cleaned, best_area


def segment(frame: np.ndarray, target: Optional[str] = None
            ) -> Tuple[Optional[np.ndarray], Optional[str]]:
    """分割目标颜色, 并只保留画面中央的连续掩码。

    - target 明确指定 → 只跑该颜色
    - 未指定 (auto): 先跑 5 种彩色, 若中央域 >= _MIN_MASK_PIXELS 就用彩色;
      否则再考虑黑色 (避免暗背景/阴影把 black 撑爆导致误判)
    """
    # 轻度模糊 BGR 后再转 HSV, 避免相机噪声击穿 inRange 的窄区间
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    def _pick(color: str) -> Tuple[np.ndarray, int]:
        raw = _mask_of_color(hsv, color)
        return _keep_central_component(raw)

    if target is not None:
        if target not in HSV_RANGES:
            return None, None
        m, area = _pick(target)
        return (m, target) if area >= _MIN_COMPONENT_PIXELS else (None, None)

    best_mask, best_name, best_count = None, None, 0
    for name in _CHROMATIC:
        m, area = _pick(name)
        if area > best_count:
            best_mask, best_name, best_count = m, name, area
    if best_count >= _MIN_MASK_PIXELS:
        return best_mask, best_name

    black_mask, black_area = _pick("black")
    if black_area > best_count:
        return black_mask, "black"
    return best_mask, best_name


# ============================================================
# 剪影 + 顶面椭圆分析
# ============================================================

@dataclass
class SilhouetteInfo:
    """剪影几何量, 全部像素单位"""
    contour: np.ndarray                  # 主轮廓
    center: Tuple[float, float]          # minAreaRect 中心 (u, v)
    axis_deg: float                      # 物料轴向 (相对图像竖直 +y 方向的角度, 顺时针为正)
    axis_vec: Tuple[float, float]        # 单位轴向量 (从底指向顶)
    perp_vec: Tuple[float, float]        # 单位垂直向量 (物料"横向")
    length_px: float                     # 沿轴向长度
    max_width_px: float                  # 垂直轴的最大宽度
    top_center: Tuple[float, float]      # 顶端点 (轴最上端)
    bottom_center: Tuple[float, float]   # 底端点
    bottom_width_px: float               # 在底端 20% 段测得的最大宽度 (= 底径投影)
    top_width_px: float                  # 在顶端 20% 段测得的最大宽度 (= 上圆柱直径投影)


def _rect_axes(rect) -> Tuple[Tuple[float, float], float, Tuple[float, float], Tuple[float, float], float, float]:
    """从 minAreaRect 提取: 中心 / 轴角 / 轴向单位向量 / 垂直向量 / 长 / 宽

    轴向 = 矩形较长边的方向。角度用相对图像竖直方向 (+y) 顺时针度数, 便于符合"物料立着"的直觉。
    """
    (cx, cy), (w, h), angle = rect
    # OpenCV: angle ∈ [-90, 0); w,h 为矩形两条边长, 无固定顺序
    if w >= h:
        long_len, short_len = w, h
        long_angle_deg = angle
    else:
        long_len, short_len = h, w
        long_angle_deg = angle + 90.0

    theta = math.radians(long_angle_deg)   # 长边在图像中相对 +x 的角度
    ax_x, ax_y = math.cos(theta), math.sin(theta)
    # 让轴向"朝上" (图像里 y 越小越上, 所以 y 分量取负)
    if ax_y > 0:
        ax_x, ax_y = -ax_x, -ax_y
    px, py = -ax_y, ax_x    # 垂直 (右手)
    # 相对图像竖直方向 (0, -1) 的顺时针角
    up_x, up_y = 0.0, -1.0
    dot = ax_x * up_x + ax_y * up_y
    cross = up_x * ax_y - up_y * ax_x
    axis_deg = math.degrees(math.atan2(cross, dot))
    return (float(cx), float(cy)), axis_deg, (ax_x, ax_y), (px, py), float(long_len), float(short_len)


def _project_contour_along_axis(contour: np.ndarray,
                                center: Tuple[float, float],
                                axis_vec: Tuple[float, float],
                                perp_vec: Tuple[float, float]
                                ) -> np.ndarray:
    """把轮廓点投到 (轴, 垂轴) 坐标系, 返回 (N, 2) 数组: 列0=沿轴距离(顶为正), 列1=垂直距离"""
    pts = contour.reshape(-1, 2).astype(np.float32)
    dx = pts[:, 0] - center[0]
    dy = pts[:, 1] - center[1]
    s = dx * axis_vec[0] + dy * axis_vec[1]      # 沿轴 (朝顶为正)
    t = dx * perp_vec[0] + dy * perp_vec[1]      # 垂轴
    return np.stack([s, t], axis=1)


def _slab_width(proj: np.ndarray, s_lo: float, s_hi: float) -> float:
    """在 [s_lo, s_hi] 轴向段里的最大垂直宽度"""
    mask = (proj[:, 0] >= s_lo) & (proj[:, 0] <= s_hi)
    if not np.any(mask):
        return 0.0
    t = proj[mask, 1]
    return float(t.max() - t.min())


def analyze_silhouette(mask: np.ndarray) -> Optional[SilhouetteInfo]:
    """从二值掩码提取剪影几何"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 200:
        return None

    rect = cv2.minAreaRect(contour)
    center, axis_deg, axis_vec, perp_vec, length_px, max_width_px = _rect_axes(rect)
    if length_px < 20 or max_width_px < 5:
        return None

    proj = _project_contour_along_axis(contour, center, axis_vec, perp_vec)
    s_top = float(proj[:, 0].max())
    s_bot = float(proj[:, 0].min())
    span = s_top - s_bot
    if span <= 1e-3:
        return None

    # 底部 25% 段的宽度 → 底径投影; 顶部 25% 段的宽度 → 上圆柱直径投影
    bot_width = _slab_width(proj, s_bot, s_bot + 0.25 * span)
    top_width = _slab_width(proj, s_top - 0.25 * span, s_top)

    top_pt = (center[0] + s_top * axis_vec[0], center[1] + s_top * axis_vec[1])
    bot_pt = (center[0] + s_bot * axis_vec[0], center[1] + s_bot * axis_vec[1])

    return SilhouetteInfo(
        contour=contour,
        center=center,
        axis_deg=axis_deg,
        axis_vec=axis_vec,
        perp_vec=perp_vec,
        length_px=length_px,
        max_width_px=max_width_px,
        top_center=top_pt,
        bottom_center=bot_pt,
        bottom_width_px=bot_width,
        top_width_px=top_width,
    )


def fit_top_ellipse(mask: np.ndarray, sil: SilhouetteInfo,
                    band_frac: float = 0.18) -> Optional[Tuple[Tuple[float, float], Tuple[float, float], float]]:
    """在剪影顶端一小段范围内, 拟合上圆柱顶面椭圆 (只在轴向 top~top-band_frac*length 之间)

    对整段掩码做 canny → 只保留位于 top band 内的边缘点 → fitEllipse。
    仅当能拿到 >= 8 个点时才尝试。
    """
    ys, xs = np.where(mask > 0)
    if xs.size < 8:
        return None
    dx = xs.astype(np.float32) - sil.center[0]
    dy = ys.astype(np.float32) - sil.center[1]
    s = dx * sil.axis_vec[0] + dy * sil.axis_vec[1]
    s_top = s.max()
    s_thresh = s_top - band_frac * sil.length_px
    keep = s >= s_thresh
    if int(np.count_nonzero(keep)) < 8:
        return None
    pts = np.stack([xs[keep], ys[keep]], axis=1).astype(np.float32)

    # 提取上带内像素的凸包边缘, 更接近上圆柱轮廓
    hull = cv2.convexHull(pts.reshape(-1, 1, 2))
    if hull.shape[0] < 5:
        return None
    try:
        return cv2.fitEllipse(hull)
    except cv2.error:
        return None


def _estimate_pitch_from_silhouette(sil: SilhouetteInfo) -> Optional[float]:
    """从累计宽度曲线找"上圆柱平台"的起点, 反算俯仰角。

    剪影自顶向下的累计最大横宽 W(d):
        0 → top_w (顶面椭圆上半, 快速上升)
        → 保持 top_w (上圆柱侧面, 平台)
        → 上升到 bottom_w (圆台段)

    "首次进入平台"的距离 d* = 顶面椭圆短轴半长 = top_w/2 * sin(pitch)。
    平台宽度 = 上圆柱真实直径投影 (即真正的 top_w)。
    """
    if sil.length_px < 20.0:
        return None
    proj = _project_contour_along_axis(sil.contour, sil.center, sil.axis_vec, sil.perp_vec)
    s_top = float(proj[:, 0].max())
    max_scan = int(min(sil.length_px * 0.6, 400))

    widths = np.zeros(max_scan + 1, dtype=np.float32)
    cur_max_t = 0.0
    for d in range(1, max_scan + 1):
        band = (proj[:, 0] <= s_top - (d - 1)) & (proj[:, 0] > s_top - d)
        if np.any(band):
            t = float(np.abs(proj[band, 1]).max())
            if t > cur_max_t:
                cur_max_t = t
        widths[d] = 2 * cur_max_t

    # 找平台: widths[d+k] - widths[d] < 1 px 内连续 k 步 → d 为平台入口
    k_stable = 3
    d_plateau: Optional[int] = None
    for d in range(4, max_scan - k_stable):
        w = widths[d]
        if w < 6:                                  # 顶端还没稳定
            continue
        if widths[d + k_stable] - w <= 1.0:        # 后续 k 步几乎不增长
            d_plateau = d
            break

    if d_plateau is None or widths[d_plateau] < 6:
        return None
    top_w = float(widths[d_plateau])
    ratio = float(np.clip(d_plateau / (top_w / 2), 0.0, 1.0))
    return math.degrees(math.asin(ratio))


# ============================================================
# 深度单点标定
# ============================================================

@dataclass
class DepthCalib:
    """单点距离标定常数

    K_bottom = bottom_width_px × distance_mm  (在拍标定图时测得)

    使用时: distance_mm = K_bottom / bottom_width_px_now
    该关系不依赖内参, 只要相机焦距不变、物料底径不变即成立。
    """
    K_bottom: float = 0.0
    ref_distance_mm: float = 0.0
    ref_bottom_width_px: float = 0.0
    ref_color: str = ""
    ref_time: str = ""

    def is_valid(self) -> bool:
        return self.K_bottom > 1.0

    @classmethod
    def load(cls, path: str) -> "DepthCalib":
        if not os.path.exists(path):
            return cls()
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        return cls(**d)

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2, ensure_ascii=False)


# ============================================================
# 主结果
# ============================================================

@dataclass
class MaterialReading:
    color: Optional[str]
    view_mode: str                           # "side" 或 "top_down"
    center_px: Tuple[float, float]
    axis_angle_deg: float
    pitch_deg: Optional[float]              # 相机俯仰 (None = side 模式下无顶面椭圆)
    distance_mm: Optional[float]            # 无标定时为 None
    max_width_px: float                     # minAreaRect 短边 (两种视角下都 ≈ 底径投影)
    bottom_width_px: float
    top_width_px: float
    silhouette_len_px: float
    top_ellipse: Optional[Tuple[Tuple[float, float], Tuple[float, float], float]]
    confidence: float

    def to_dict(self) -> dict:
        return {
            "color": self.color,
            "view_mode": self.view_mode,
            "center_px": self.center_px,
            "axis_angle_deg": self.axis_angle_deg,
            "pitch_deg": self.pitch_deg,
            "distance_mm": self.distance_mm,
            "max_width_px": self.max_width_px,
            "bottom_width_px": self.bottom_width_px,
            "top_width_px": self.top_width_px,
            "silhouette_len_px": self.silhouette_len_px,
            "confidence": self.confidence,
        }


# ============================================================
# 检测器
# ============================================================

class MaterialDetector:
    def __init__(self, spec: MaterialSpec, calib: Optional[DepthCalib] = None):
        self.spec = spec
        self.calib = calib or DepthCalib()

    # 侧视 vs 正俯视判据: 剪影长/短边比 < 此值 → 正俯视 (推导见文件头 pitch 表)
    ASPECT_TOPDOWN_THRESHOLD = 1.10

    def detect(self, frame: np.ndarray, target_color: Optional[str] = None
               ) -> Tuple[Optional[MaterialReading], Optional[np.ndarray]]:
        mask, color = segment(frame, target_color)
        if mask is None or cv2.countNonZero(mask) < 300:
            return None, mask

        sil = analyze_silhouette(mask)
        if sil is None:
            return None, mask

        aspect = sil.length_px / max(sil.max_width_px, 1.0)

        if aspect < self.ASPECT_TOPDOWN_THRESHOLD:
            return self._make_topdown_reading(sil, color), mask
        return self._make_side_reading(sil, mask, color), mask

    def _distance_from(self, width_px: float) -> Optional[float]:
        if self.calib.is_valid() and width_px > 1.0:
            return self.calib.K_bottom / width_px
        return None

    def _make_side_reading(self, sil: SilhouetteInfo, mask: np.ndarray,
                           color: Optional[str]) -> MaterialReading:
        top_ellipse = fit_top_ellipse(mask, sil)
        pitch_deg = _estimate_pitch_from_silhouette(sil)
        # 平台法失败 → 回退用顶面椭圆短/长比给 pitch
        if pitch_deg is None and top_ellipse is not None:
            (_, _), (e_w, e_h), _ = top_ellipse
            major, minor = max(e_w, e_h), min(e_w, e_h)
            if major > 1.0:
                pitch_deg = math.degrees(math.asin(min(minor / major, 1.0)))

        # 距离用 minAreaRect 短边 (= 底径的横向投影), 比 slab 更稳
        distance_mm = self._distance_from(sil.max_width_px)

        conf = 0.4
        if sil.bottom_width_px > sil.top_width_px * 1.05:
            conf += 0.2                             # 底比顶宽, 符合截头锥
        if top_ellipse is not None:
            conf += 0.2
        aspect = sil.length_px / max(sil.max_width_px, 1.0)
        # 侧视理论 aspect ∈ [1.10, 1.44], 平视 ≈ 1.20
        if 1.05 <= aspect <= 1.70:
            conf += 0.1
        if distance_mm is not None:
            conf += 0.1

        return MaterialReading(
            color=color,
            view_mode="side",
            center_px=sil.center,
            axis_angle_deg=sil.axis_deg,
            pitch_deg=pitch_deg,
            distance_mm=distance_mm,
            max_width_px=sil.max_width_px,
            bottom_width_px=sil.bottom_width_px,
            top_width_px=sil.top_width_px,
            silhouette_len_px=sil.length_px,
            top_ellipse=top_ellipse,
            confidence=float(np.clip(conf, 0.0, 1.0)),
        )

    def _make_topdown_reading(self, sil: SilhouetteInfo,
                              color: Optional[str]) -> MaterialReading:
        """正俯视: 剪影近圆, minAreaRect 长短边都 ≈ 底径投影。
        axis 无意义 (在物料底面里旋转), 置为 0; pitch 直接 90°。
        距离仍用 max_width_px = 底径投影。
        """
        # 用两条边的均值当底径投影, 抗抖
        outer_diam_px = 0.5 * (sil.length_px + sil.max_width_px)
        distance_mm = self._distance_from(outer_diam_px)

        conf = 0.5
        aspect = sil.length_px / max(sil.max_width_px, 1.0)
        if aspect < 1.05:
            conf += 0.2                              # 越接近正圆越可信
        if distance_mm is not None:
            conf += 0.2

        return MaterialReading(
            color=color,
            view_mode="top_down",
            center_px=sil.center,
            axis_angle_deg=0.0,
            pitch_deg=90.0,
            distance_mm=distance_mm,
            max_width_px=outer_diam_px,
            bottom_width_px=outer_diam_px,          # 顶视下 slab 意义不大, 复用外径
            top_width_px=outer_diam_px,
            silhouette_len_px=sil.length_px,
            top_ellipse=None,
            confidence=float(np.clip(conf, 0.0, 1.0)),
        )


# ============================================================
# 可视化
# ============================================================

_COLOR_BGR = {
    "red":        (60, 60, 240),
    "yellow":     (0, 220, 240),
    "green":      (80, 200, 80),
    "blue":       (230, 120, 60),
    "light_blue": (240, 220, 120),
    "black":      (60, 60, 60),
}


def draw_reading(frame: np.ndarray, reading: MaterialReading, spec: MaterialSpec) -> np.ndarray:
    out = frame.copy()
    cx, cy = int(round(reading.center_px[0])), int(round(reading.center_px[1]))

    # 中心
    cv2.circle(out, (cx, cy), 4, (0, 255, 255), -1)

    if reading.view_mode == "top_down":
        # 正俯视: 画外径 (底径投影) + 内径估计 (顶径投影 = 外径 × 30/50)
        r_out = int(round(reading.max_width_px / 2))
        r_in  = int(round(r_out * spec.top_d / spec.bottom_d))
        cv2.circle(out, (cx, cy), r_out, (0, 255, 0), 2)
        cv2.circle(out, (cx, cy), r_in,  (0, 200, 255), 2)
    else:
        # 侧视: 轴向箭头 + 顶面椭圆
        L = reading.silhouette_len_px / 2
        theta = math.radians(reading.axis_angle_deg)
        ax_x, ax_y = math.sin(theta), -math.cos(theta)
        top = (int(round(cx + ax_x * L)), int(round(cy + ax_y * L)))
        bot = (int(round(cx - ax_x * L)), int(round(cy - ax_y * L)))
        cv2.arrowedLine(out, bot, top, (0, 255, 0), 2, tipLength=0.15)
        if reading.top_ellipse is not None:
            cv2.ellipse(out, reading.top_ellipse, (0, 200, 255), 2)

    # HUD
    y = 24
    def _put(txt, col=(255, 255, 255)):
        nonlocal y
        cv2.putText(out, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(out, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)
        y += 22

    color_bgr = _COLOR_BGR.get(reading.color or "", (255, 255, 255))
    _put(f"color: {reading.color or '-'}    view: {reading.view_mode}", color_bgr)
    _put(f"center(u,v): ({reading.center_px[0]:.1f}, {reading.center_px[1]:.1f})")
    if reading.view_mode == "top_down":
        _put("axis angle: N/A (top-down)")
    else:
        _put(f"axis angle: {reading.axis_angle_deg:+.1f} deg (from image vertical)")
    if reading.pitch_deg is not None:
        _put(f"camera pitch: {reading.pitch_deg:.1f} deg (0=level, 90=top-down)")
    else:
        _put("camera pitch: N/A (no top ellipse)")
    if reading.distance_mm is not None:
        _put(f"distance: {reading.distance_mm:.1f} mm", (0, 255, 0))
    else:
        _put("distance: N/A (run --calibrate first)", (0, 200, 255))
    _put(f"bottom width: {reading.bottom_width_px:.1f} px    "
         f"top width: {reading.top_width_px:.1f} px")
    _put(f"silhouette len: {reading.silhouette_len_px:.1f} px    "
         f"conf: {reading.confidence:.2f}",
         (0, 255, 0) if reading.confidence > 0.7 else (0, 200, 255))

    # 参考: 物料标称长宽比
    _put(f"spec: dia {spec.bottom_d:.0f} / {spec.top_d:.0f} mm, H {spec.total_h:.0f} mm",
         (180, 180, 180))
    return out


# ============================================================
# 合成图像 (用于 --test)
# ============================================================

def _render_topdown(spec: MaterialSpec, scale: float, roll_deg: float,
                    image_size: Tuple[int, int],
                    color_bgr: Tuple[int, int, int],
                    img: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
    """正俯视合成: 底径 = 大圆, 顶径 = 内小圆 (稍暗色)。roll 对同心圆无视觉影响。"""
    H, W = image_size
    cx, cy = W // 2, H // 2
    r_out = int(round(spec.bottom_d * scale / 2))
    r_in  = int(round(spec.top_d    * scale / 2))
    inner_color = tuple(int(c * 0.7) for c in color_bgr)
    cv2.circle(img, (cx, cy), r_out, color_bgr, -1)
    cv2.circle(img, (cx, cy), r_in,  inner_color, -1)
    _ = roll_deg
    noise = np.random.randint(-8, 8, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return img, (W, H)


def render_synthetic(spec: MaterialSpec,
                     distance_mm: float,
                     camera_pitch_deg: float,
                     roll_deg: float,
                     image_size: Tuple[int, int] = (720, 960),
                     color_bgr: Tuple[int, int, int] = (60, 60, 240),
                     f_pixels: float = 900.0,
                     ) -> Tuple[np.ndarray, Tuple[int, int]]:
    """图像空间直接合成物料图, 供 --test 使用。

    简化模型: 平视图 (物料竖直在图像里) → 应用 camera_pitch 压缩沿轴方向 (cos)
    并把顶面画成椭圆 (短/长 = sin(pitch)) → 整图绕中心 roll_deg。
    """
    H, W = image_size
    img = np.full((H, W, 3), 200, dtype=np.uint8)   # 浅灰背景

    scale = f_pixels / max(distance_mm, 1.0)         # mm → px

    # 近正俯视 → 走同心圆分支 (侧面 polygon 在 pitch≈90° 时退化)
    if camera_pitch_deg >= 80.0:
        return _render_topdown(spec, scale, roll_deg, image_size, color_bgr, img)

    pitch = math.radians(camera_pitch_deg)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)

    # 平视基准尺寸 (px)
    bot_w_full = spec.bottom_d * scale
    top_w_full = spec.top_d * scale
    total_len_full = spec.total_h * scale

    # 相机俯仰使沿轴方向被 cos(p) 压缩; 横向不变
    bot_w = bot_w_full
    top_w = top_w_full
    total_len = total_len_full * cos_p

    # 三段沿轴长度
    seg_bot = spec.bottom_cyl_h * scale * cos_p
    seg_frust = spec.frustum_h  * scale * cos_p
    seg_top  = spec.top_cyl_h   * scale * cos_p
    _ = seg_bot + seg_frust + seg_top   # = total_len

    cx, cy = W / 2, H / 2
    half = total_len / 2

    # 侧面剪影 (轴向向上 +y, 图像 y 越小越上; 用 v 坐标)
    # 顶面椭圆的圆心在梯形上边缘之上 top_w/2*sin(p) 处 (让椭圆完整可见)
    top_ellipse_minor = max(1.0, top_w / 2 * sin_p)
    v_bot = cy + half
    v_frust_bot = v_bot - seg_bot
    v_frust_top = v_frust_bot - seg_frust
    v_top = v_frust_top - seg_top          # 上圆柱顶面圆心
    v_top_pgon = v_top + top_ellipse_minor  # 梯形顶边 = 椭圆下沿

    polygon = np.array([
        [cx - bot_w / 2, v_bot],
        [cx + bot_w / 2, v_bot],
        [cx + bot_w / 2, v_frust_bot],
        [cx + top_w / 2, v_frust_top],
        [cx + top_w / 2, v_top_pgon],
        [cx - top_w / 2, v_top_pgon],
        [cx - top_w / 2, v_frust_top],
        [cx - bot_w / 2, v_frust_bot],
    ], dtype=np.float32)

    top_ellipse_center = (cx, v_top)
    top_ellipse_axes   = (top_w / 2, top_ellipse_minor)

    # 底面椭圆的下半 (若 pitch>0, 底面可见一点弧) — 只是让剪影更真, 可选
    bot_ellipse_center = (cx, v_bot)
    bot_ellipse_axes   = (bot_w / 2, max(1.0, bot_w / 2 * sin_p))

    # roll 变换
    roll = math.radians(roll_deg)
    cos_r, sin_r = math.cos(roll), math.sin(roll)

    def rot(pt):
        du = pt[0] - cx
        dv = pt[1] - cy
        return (du * cos_r - dv * sin_r + cx,
                du * sin_r + dv * cos_r + cy)

    poly_r = np.array([rot(p) for p in polygon], dtype=np.int32)
    cv2.fillPoly(img, [poly_r], color_bgr)

    # 顶面椭圆盖 (稍深色区分)
    top_color = tuple(int(c * 0.7) for c in color_bgr)
    cv2.ellipse(img, (int(round(rot(top_ellipse_center)[0])),
                      int(round(rot(top_ellipse_center)[1]))),
                (int(round(top_ellipse_axes[0])), int(round(top_ellipse_axes[1]))),
                roll_deg, 0, 360, top_color, -1)

    # 底面椭圆下半弧 (可见部分, 只画下半)
    cv2.ellipse(img, (int(round(rot(bot_ellipse_center)[0])),
                      int(round(rot(bot_ellipse_center)[1]))),
                (int(round(bot_ellipse_axes[0])), int(round(bot_ellipse_axes[1]))),
                roll_deg, 0, 180, color_bgr, -1)

    # 一点噪声
    noise = np.random.randint(-8, 8, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    return img, (W, H)


# ============================================================
# 单张图片 / 合成测试
# ============================================================

def run_image(path: str, spec: MaterialSpec, calib: DepthCalib,
              target_color: Optional[str], save_path: Optional[str]) -> None:
    frame = cv2.imread(path)
    if frame is None:
        print(f"无法读取图像: {path}")
        return
    detector = MaterialDetector(spec, calib)
    reading, mask = detector.detect(frame, target_color)
    if reading is None:
        print("未检测到物料 (检查颜色范围 / 光照)")
        if save_path and mask is not None:
            cv2.imwrite(save_path, mask)
        return
    print("== 检测结果 ==")
    for k, v in reading.to_dict().items():
        print(f"  {k}: {v}")
    display = draw_reading(frame, reading, spec)
    out = save_path or "material_output.png"
    cv2.imwrite(out, display)
    print(f"标注图已写入 {out}")


def test_mode(spec: MaterialSpec) -> None:
    print("== 合成测试 ==")
    calib = DepthCalib()          # 无标定, 距离显示 N/A

    for i, (dist, pitch, roll, color_name) in enumerate([
        (300.0, 25.0, 0.0,  "red"),
        (450.0, 40.0, 15.0, "yellow"),
        (250.0, 10.0, -8.0, "blue"),
    ]):
        color_bgr = _COLOR_BGR[color_name]
        img, _ = render_synthetic(spec, dist, pitch, roll, color_bgr=color_bgr)
        detector = MaterialDetector(spec, calib)
        reading, mask = detector.detect(img)
        if reading is None:
            print(f"[case {i}] 未检测到")
            continue
        display = draw_reading(img, reading, spec)
        out = f"material_test_{i}_{color_name}.png"
        cv2.imwrite(out, display)
        print(f"[case {i}] dist_gt={dist}mm pitch_gt={pitch}° roll_gt={roll}° | "
              f"detected color={reading.color} axis={reading.axis_angle_deg:+.1f}° "
              f"pitch={reading.pitch_deg}° conf={reading.confidence:.2f}   -> {out}")

    # 单点标定回归测试
    print("\n== 单点标定回归 ==")
    ref_dist = 300.0
    ref_img, _ = render_synthetic(spec, ref_dist, 25.0, 0.0, color_bgr=_COLOR_BGR["red"])
    det = MaterialDetector(spec)
    reading, _ = det.detect(ref_img, "red")
    if reading is None:
        print("标定帧检测失败")
        return
    calib.K_bottom = reading.bottom_width_px * ref_dist
    calib.ref_distance_mm = ref_dist
    calib.ref_bottom_width_px = reading.bottom_width_px
    calib.ref_color = "red"
    print(f"K_bottom = {calib.K_bottom:.1f} (ref width {reading.bottom_width_px:.1f} px @ {ref_dist} mm)")

    det2 = MaterialDetector(spec, calib)
    for dist_gt in [200.0, 300.0, 400.0, 550.0]:
        img, _ = render_synthetic(spec, dist_gt, 25.0, 0.0, color_bgr=_COLOR_BGR["red"])
        r, _ = det2.detect(img, "red")
        if r and r.distance_mm is not None:
            err = r.distance_mm - dist_gt
            print(f"  gt={dist_gt:.0f} mm -> est={r.distance_mm:.1f} mm "
                  f"(err {err:+.1f} mm, {err/dist_gt*100:+.1f}%)")


# ============================================================
# 摄像头 + 交互式标定
# ============================================================

def run_camera(camera_index: int, spec: MaterialSpec, calib: DepthCalib,
               calib_path: str,
               target_color: Optional[str],
               calib_distance_mm: Optional[float],
               flip: bool) -> None:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"无法打开摄像头 {camera_index}")
        return
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"分辨率 {w}x{h}")
    if calib.is_valid():
        print(f"深度标定: K_bottom={calib.K_bottom:.1f}  (ref {calib.ref_distance_mm} mm)")
    else:
        print("无深度标定. 若需距离输出, 请加 --calibrate <dist_mm> 后按 'k' 采样.")

    detector = MaterialDetector(spec, calib)
    colors = [target_color] if target_color else [None] + list(HSV_RANGES.keys())
    color_idx = 0

    print("按键: q 退出 | c 切颜色 | s 存图 | k 标定 (需要 --calibrate)")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if flip:
            frame = cv2.flip(frame, 1)

        reading, mask = detector.detect(frame, colors[color_idx])

        if reading is not None:
            display = draw_reading(frame, reading, spec)
        else:
            display = frame.copy()
            cv2.putText(display, f"No material (color={colors[color_idx] or 'auto'})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 右下角贴掩码缩略图
        if mask is not None:
            small = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                               (w // 4, h // 4))
            display[h - h // 4:h, w - w // 4:w] = small

        cv2.imshow("material_detector", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            color_idx = (color_idx + 1) % len(colors)
            print(f"目标颜色 -> {colors[color_idx] or 'auto'}")
        elif key == ord('s'):
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"material_{ts}.png", display)
            if reading is not None:
                with open(f"material_{ts}.json", "w", encoding="utf-8") as f:
                    json.dump(reading.to_dict(), f, indent=2, ensure_ascii=False)
            print(f"已保存 material_{ts}.png")
        elif key == ord('k'):
            if calib_distance_mm is None:
                print("按 k 无效: 请用 --calibrate <dist_mm> 启动")
                continue
            if reading is None:
                print("按 k 无效: 当前帧未检测到物料")
                continue
            calib.K_bottom = reading.bottom_width_px * calib_distance_mm
            calib.ref_distance_mm = calib_distance_mm
            calib.ref_bottom_width_px = reading.bottom_width_px
            calib.ref_color = reading.color or ""
            calib.ref_time = time.strftime("%Y-%m-%d %H:%M:%S")
            calib.save(calib_path)
            detector.calib = calib
            print(f"标定完成: K_bottom={calib.K_bottom:.1f}  已写入 {calib_path}")

    cap.release()
    cv2.destroyAllWindows()


# ============================================================
# CLI
# ============================================================

def _setup_stdout_utf8() -> None:
    """让 Windows GBK 控制台不至于因 →/∈/… 等字符崩溃。"""
    for stream_name in ("stdout", "stderr"):
        stream = getattr(__import__("sys"), stream_name, None)
        if stream is None:
            continue
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def main() -> None:
    _setup_stdout_utf8()
    p = argparse.ArgumentParser(description="物料识别 — 圆柱+圆台+圆柱, 不需要相机内参")
    p.add_argument("--image", type=str, default=None, help="处理单张图片")
    p.add_argument("--camera", type=int, default=None, help="实时摄像头索引")
    p.add_argument("--test", action="store_true", help="合成图像自检")
    p.add_argument("--color", type=str, default=None,
                   choices=list(HSV_RANGES.keys()),
                   help="目标颜色 (省略=自动取最大掩码)")
    p.add_argument("--calibrate", type=float, default=None, metavar="DIST_MM",
                   help="启用摄像头交互式标定: 在此距离下按 'k' 采一次样")
    p.add_argument("--calib-file", type=str, default="depth_calib.json",
                   help="深度标定文件")
    p.add_argument("--save", type=str, default=None, help="--image 模式下的输出图像路径")
    p.add_argument("--no-flip", action="store_true", help="摄像头模式不做水平翻转")
    # 物料尺寸 (与二段描述保持一致的默认值)
    p.add_argument("--bottom-cyl-h", type=float, default=18.0)
    p.add_argument("--bottom-r",     type=float, default=25.0)
    p.add_argument("--frustum-h",    type=float, default=34.0)
    p.add_argument("--top-r",        type=float, default=15.0)
    p.add_argument("--top-cyl-h",    type=float, default=8.0)
    args = p.parse_args()

    spec = MaterialSpec(
        bottom_cyl_h=args.bottom_cyl_h,
        bottom_r=args.bottom_r,
        frustum_h=args.frustum_h,
        top_r=args.top_r,
        top_cyl_h=args.top_cyl_h,
    )
    print(f"[spec] 下圆柱 h={spec.bottom_cyl_h} r={spec.bottom_r} | "
          f"圆台 h={spec.frustum_h} ({spec.bottom_r}→{spec.top_r}) | "
          f"上圆柱 h={spec.top_cyl_h} r={spec.top_r} | 总高 {spec.total_h}mm")

    calib = DepthCalib.load(args.calib_file)
    if calib.is_valid():
        print(f"[calib] loaded {args.calib_file}: K_bottom={calib.K_bottom:.1f} "
              f"(ref {calib.ref_distance_mm}mm)")

    if args.test:
        test_mode(spec)
    elif args.image is not None:
        run_image(args.image, spec, calib, args.color, args.save)
    elif args.camera is not None:
        run_camera(args.camera, spec, calib, args.calib_file,
                   args.color, args.calibrate, flip=not args.no_flip)
    else:
        p.print_help()


if __name__ == "__main__":
    main()