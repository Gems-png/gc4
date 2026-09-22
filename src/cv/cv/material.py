"""物料识别算法库 — 圆台组合体, 无内参依赖 (缩编为纯 import 模块)

由 MainCam 通过 `from cv import material` 引入, 经 self.detect_material() 调用。
原 CLI / 摄像头交互 / 合成图像自检部分已删除。

物料几何 (回转体, 单位 mm):
    下圆柱   r=25, h=8            z ∈ [0, 18]
    圆台     r: 25→15, h=44       z ∈ [18, 52]
    上圆柱   r=15, h=8            z ∈ [52, 60]
    总高 60, 底径 50, 顶径 30

核心算法 (不需要内参):
    1. HSV 颜色分割 → 二值掩码
    2. 中央连续域优先 (connectedComponents), 抑制画面边缘干扰
    3. 掩码 minAreaRect → 剪影长短边、中心、粗略轴向
    4. 剪影长/短边比自动分派: side_view / top_down
    5. 距离: D = K / max_width_px, K = 单点标定常数 (DepthCalib)
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, asdict
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
    "yellow":     [((10, 30, 110), (33, 255, 255))],
    "green":      [((50, 90, 60),   (80, 255, 255))],
    "blue":       [((100, 55, 80), (140, 255, 255))],
    "light_blue": [((86, 30, 130),  (100, 255, 255))],
    "black":      [((0, 0, 0),      (180, 50, 150))],
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
    axis_deg: float                      # 物料轴向 (相对图像竖直方向的角度, 顺时针为正)
    axis_vec: Tuple[float, float]        # 单位轴向量 (从底指向顶)
    perp_vec: Tuple[float, float]        # 单位垂直向量 (物料"横向")
    length_px: float                     # 沿轴向长度
    max_width_px: float                  # 垂直轴的最大宽度
    top_center: Tuple[float, float]      # 顶端点 (轴最上端)
    bottom_center: Tuple[float, float]   # 底端点
    bottom_width_px: float               # 在底端 25% 段测得的最大宽度 (= 底径投影)
    top_width_px: float                  # 在顶端 25% 段测得的最大宽度 (= 上圆柱直径投影)


def _rect_axes(rect) -> Tuple[Tuple[float, float], float, Tuple[float, float], Tuple[float, float], float, float]:
    """从 minAreaRect 提取: 中心 / 轴角 / 轴向单位向量 / 垂直向量 / 长 / 宽"""
    (cx, cy), (w, h), angle = rect
    if w >= h:
        long_len, short_len = w, h
        long_angle_deg = angle
    else:
        long_len, short_len = h, w
        long_angle_deg = angle + 90.0

    theta = math.radians(long_angle_deg)
    ax_x, ax_y = math.cos(theta), math.sin(theta)
    # 让轴向"朝上" (图像里 y 越小越上)
    if ax_y > 0:
        ax_x, ax_y = -ax_x, -ax_y
    px, py = -ax_y, ax_x    # 垂直 (右手)
    # 相对图像竖直方向 (0, -1) 的顺时针角
    dot = ax_x * 0.0 + ax_y * (-1.0)
    cross = 0.0 * ax_y - (-1.0) * ax_x
    axis_deg = math.degrees(math.atan2(cross, dot))
    return (float(cx), float(cy)), axis_deg, (ax_x, ax_y), (px, py), float(long_len), float(short_len)


def _project_contour_along_axis(contour: np.ndarray,
                                center: Tuple[float, float],
                                axis_vec: Tuple[float, float],
                                perp_vec: Tuple[float, float]
                                ) -> np.ndarray:
    """把轮廓点投到 (轴, 垂轴) 坐标系: 列0=沿轴距离(顶为正), 列1=垂直距离"""
    pts = contour.reshape(-1, 2).astype(np.float32)
    dx = pts[:, 0] - center[0]
    dy = pts[:, 1] - center[1]
    s = dx * axis_vec[0] + dy * axis_vec[1]
    t = dx * perp_vec[0] + dy * perp_vec[1]
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
    """在剪影顶端一小段范围内, 拟合上圆柱顶面椭圆。仅当能拿到 >= 8 个点时才尝试。"""
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

    hull = cv2.convexHull(pts.reshape(-1, 1, 2))
    if hull.shape[0] < 5:
        return None
    try:
        return cv2.fitEllipse(hull)
    except cv2.error:
        return None


def _estimate_pitch_from_silhouette(sil: SilhouetteInfo) -> Optional[float]:
    """从累计宽度曲线找"上圆柱平台"的起点, 反算俯仰角。"""
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

    k_stable = 3
    d_plateau: Optional[int] = None
    for d in range(4, max_scan - k_stable):
        w = widths[d]
        if w < 6:
            continue
        if widths[d + k_stable] - w <= 1.0:
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
    """单点距离标定: distance_mm = K_bottom / bottom_width_px_now"""
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
    pitch_deg: Optional[float]
    distance_mm: Optional[float]
    max_width_px: float
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

    # 侧视 vs 正俯视判据: 剪影长/短边比 < 此值 → 正俯视
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

        distance_mm = self._distance_from(sil.max_width_px)

        conf = 0.4
        if sil.bottom_width_px > sil.top_width_px * 1.05:
            conf += 0.2
        if top_ellipse is not None:
            conf += 0.2
        aspect = sil.length_px / max(sil.max_width_px, 1.0)
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
        """正俯视: 剪影近圆, pitch=90°, 距离用外径投影。"""
        outer_diam_px = 0.5 * (sil.length_px + sil.max_width_px)
        distance_mm = self._distance_from(outer_diam_px)

        conf = 0.5
        aspect = sil.length_px / max(sil.max_width_px, 1.0)
        if aspect < 1.05:
            conf += 0.2
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
            bottom_width_px=outer_diam_px,
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

    cv2.circle(out, (cx, cy), 4, (0, 255, 255), -1)

    if reading.view_mode == "top_down":
        r_out = int(round(reading.max_width_px / 2))
        r_in  = int(round(r_out * spec.top_d / spec.bottom_d))
        cv2.circle(out, (cx, cy), r_out, (0, 255, 0), 2)
        cv2.circle(out, (cx, cy), r_in,  (0, 200, 255), 2)
    else:
        L = reading.silhouette_len_px / 2
        theta = math.radians(reading.axis_angle_deg)
        ax_x, ax_y = math.sin(theta), -math.cos(theta)
        top = (int(round(cx + ax_x * L)), int(round(cy + ax_y * L)))
        bot = (int(round(cx - ax_x * L)), int(round(cy - ax_y * L)))
        cv2.arrowedLine(out, bot, top, (0, 255, 0), 2, tipLength=0.15)
        if reading.top_ellipse is not None:
            cv2.ellipse(out, reading.top_ellipse, (0, 200, 255), 2)

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
        _put("distance: N/A (calibrate DepthCalib first)", (0, 200, 255))
    _put(f"bottom width: {reading.bottom_width_px:.1f} px    "
         f"top width: {reading.top_width_px:.1f} px")
    _put(f"silhouette len: {reading.silhouette_len_px:.1f} px    "
         f"conf: {reading.confidence:.2f}",
         (0, 255, 0) if reading.confidence > 0.7 else (0, 200, 255))
    _put(f"spec: dia {spec.bottom_d:.0f} / {spec.top_d:.0f} mm, H {spec.total_h:.0f} mm",
         (180, 180, 180))
    return out
