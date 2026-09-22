# AprilTag (36h11) 识别 + 位姿 —— 纯函数模块
"""认 AprilTag(36h11) 方块, 算出 tag 在相机系下的位置 (mm)。

⚠ 跟 qrcode.py 是两回事, 别混: 这边是要标定内参、要算位姿的 AprilTag (给视觉自定义
控制器那套用); 那边认的是规则里印的普通二维码 tag。两个模块各管各的, 互不 import。

原先这些代码散在 MainCam 里 (更早是 cam_pos / Apriltag_pose / Apriltag_image_pub 三个
节点), 现在整块搬进来, 连参数一起 —— 仿 Minit.open_cap 的封装: 一个功能加它的**全部
参数**都封在这一个文件里。节点那边一句就够:

    tracker = TagTracker(method='similar')        # 或者什么都不传, 用默认
    pose = tracker.track(frame)                   # 认到 -> TagPose, 没认到 -> None

两种位姿方法 (原来的两个节点, 现在只是 TagTracker 的一个参数):
    'similar'  相似三角形测距 —— 只用 fx/fy/cx/cy, 由 tag 的像素边长反算距离,
               不依赖 solvePnP 的旋转解算, 距离更稳, 但只给位置不给姿态。
    'pnp'      solvePnP 全位姿 —— 位置 + 姿态, 并且在第一次认到 tag 时把世界坐标系
               锁定在 tag 上 (之后相机怎么动, 报的都是"相对 tag 的世界系"下的量)。

标定文件 (内参) 由**模块自己解析** (default_calib_file): 先找装好的
share/cv/config/gc480p.json, 找不到再回头找源码树的 ../config。调用方不用传路径,
真要换一份就 params 里写 calib_file。

自检: python3 apriltag.py —— 合成 AprilTag 图自己认一遍, 不碰相机也不用 ROS。
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, replace
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

DICT_NAME = 'DICT_APRILTAG_36h11'      # 赛场上用的字典, 换 tag 家族就改这儿
DEFAULT_TAG_SIZE_MM = 40.0             # tag 黑框边长 (mm), 测距的基准, 得拿尺子量印出来的
DEFAULT_TAG_ID = 1
DEFAULT_METHOD = 'pnp'
CALIB_NAME = 'gc480p.json'             # 主相机 (2M) 的标定文件
METHODS = ('pnp', 'similar')

# 立体目标点在 tag 系下的坐标 (tag 边长归一化, 单位是"几个 tag 边长")
_OBJ_UNIT = np.array([[-0.5,  0.5, 0.0],
                      [ 0.5,  0.5, 0.0],
                      [ 0.5, -0.5, 0.0],
                      [-0.5, -0.5, 0.0]], dtype=np.float64)


# ============================================================
# 标定文件: 自己找
# ============================================================

def default_calib_file() -> str:
    """标定文件在哪儿 —— 模块自己解析, 返回路径; 找不到返回 ''。

    先问 ament (装好的包: share/cv/config/), 再回头看源码树 (src/cv/config/)。
    后者是给"直接 py apriltag.py 自检"用的, 上机跑永远走前者。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('cv'), 'config', CALIB_NAME)
        if os.path.exists(p):
            return p
    except Exception:
        # 没装 ROS / 没 source 的机器上 import 就会炸, 这很正常 —— 往下走源码树那条
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.normpath(os.path.join(here, os.pardir, 'config', CALIB_NAME))
    return p if os.path.exists(p) else ''


def load_intrinsics(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """读标定 json, 返回 (内参矩阵 3x3, 畸变系数)。格式不对就直接报错, 别带病往下跑。"""
    if not path or not os.path.exists(path):
        raise FileNotFoundError(
            f'内参文件 {path or "(没找到)"} 不存在 —— 先标定, 或者把 calib_file 指对。')
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if 'camera_matrix' not in data:
        raise ValueError(f'{path} 里没有 camera_matrix, 不是有效的内参文件。')
    inmtx = np.asarray(data['camera_matrix'], dtype=np.float64)
    distortion = np.asarray(data.get('distortion_coefficients', [0, 0, 0, 0, 0]),
                            dtype=np.float64).ravel()
    return inmtx, distortion


# ============================================================
# 参数 (全封在这里)
# ============================================================

@dataclass(frozen=True)
class AprilTagParams:
    """认 AprilTag 加算位姿的全部参数。

    method      'pnp' 或 'similar' (见文件头)。写 '' = 用默认。
    tag_size_mm tag 黑框的真实边长。印出来是 40mm 就写 40 —— 这个数直接决定测距准不准,
                猜错就是系统性比例误差 (tag_size 写 50 而实际 40, 距离就全偏 25%)。
    tag_id      认哪个 id。<0 = 不挑, 认画面里最大的那个 (多个 tag 时用)。
    calib_file  内参 json。'' = 自动找 (default_calib_file)。
    kalman      pnp 法要不要对 X/Y 做卡尔曼滤波 (Z 不过滤, 直接用)。
    goal_gain   goal 的 x/y 乘这个系数 —— 原代码写死 ×2, 因为相机不方便左右挪,
                放大一点响应快些; z 不放大。
    k_gain      similar 法里像素偏移转 mm 的放大系数 (原代码写死 5)。
    axis_len_mm / axis_thickness  pnp 调试图里那三根坐标轴的长度和线宽。
    corner_refine  角点亚像素精化。画质很差时它会挑出野点, 可以关掉试试。
    """
    method: str = ''                     # '' = 用 DEFAULT_METHOD
    tag_size_mm: float = DEFAULT_TAG_SIZE_MM
    tag_id: int = DEFAULT_TAG_ID
    calib_file: str = ''                 # '' = 自动找
    kalman: bool = True
    goal_gain: float = 2.0
    k_gain: float = 5.0
    axis_len_mm: float = 40.0
    axis_thickness: int = 3
    corner_refine: bool = True

    # ---- 下面几个是 'auto' 展开出来的结果, 不是给调用方填的 ----

    def resolved(self) -> 'AprilTagParams':
        """把 'auto' 展开成实际值 ('' → 默认; 找不到标定文件就报错)。

        幂等: 展开过的再调一次结果一样。circle 那边也是这个套路。
        """
        method = (self.method or DEFAULT_METHOD).lower()
        if method not in METHODS:
            raise ValueError(f'method 只能填 {METHODS} 之一, 收到 {self.method!r}')
        calib = self.calib_file or default_calib_file()
        if not calib:
            raise FileNotFoundError(
                f'找不到标定文件 {CALIB_NAME} (share/cv/config 和源码树 config 都没有) —— '
                f'用 AprilTag 位姿必须先有内参。')
        size = float(self.tag_size_mm)
        if size <= 0:
            raise ValueError(f'tag_size_mm 必须 > 0 (是 tag 黑框的实际边长 mm), 收到 {size}')
        return replace(self, method=method, calib_file=calib, tag_size_mm=size)

    @property
    def wants_largest(self) -> bool:
        """没有指定 id —— 认画面里最大的那个。"""
        return self.tag_id is None or int(self.tag_id) < 0

    def describe(self) -> str:
        return (f'方法={self.method} tag边长={self.tag_size_mm:g}mm '
                f'id={"最大那个" if self.wants_largest else self.tag_id} '
                f'卡尔曼={"开" if self.kalman else "关"} '
                f'goal放大={self.goal_gain:g} k={self.k_gain:g}')


def default_params(**overrides) -> AprilTagParams:
    """一套默认参数; 想改哪几个就传哪几个 (名字写错会直接 TypeError, 不静默忽略)。"""
    p = AprilTagParams()
    return replace(p, **overrides) if overrides else p


# ============================================================
# 结果
# ============================================================

@dataclass(frozen=True)
class TagCorners:
    """一帧里认到的一个 tag (还没算位姿)。corners 是 4x2 的图像像素角点。"""
    tag_id: int
    corners: np.ndarray

    @property
    def center(self) -> Tuple[float, float]:
        u, v = self.corners.reshape(-1, 2).mean(axis=0)
        return float(u), float(v)

    @property
    def area(self) -> float:
        """多边形面积 (像素²) —— 挑"最大的那个"就是比这个。"""
        p = self.corners.reshape(-1, 2)
        x, y = p[:, 0], p[:, 1]
        return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))

    @property
    def edge_px(self) -> float:
        """四条边的平均像素边长 (similar 法测距用的就是这个 s)。"""
        c = self.corners.reshape(-1, 2)
        return float(sum(np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)) / 4.0)

    def describe(self) -> str:
        u, v = self.center
        return f'id={self.tag_id} 中心({u:.0f},{v:.0f}) 边长{self.edge_px:.1f}px'


@dataclass(frozen=True)
class TagPose:
    """一帧里认到的那个 tag, 连同算出来的位姿 (发布用的就是这些字段)。

    x/y/z        相机系下的位置 (mm), pnp 法这里是卡尔曼滤波之后的。x 向右, y 向下,
                 z 是离相机的距离 (正数)。
    goal_mm      发给 /goal_position 的值 —— 两个方法的历史口径不一样, 别自己乘:
                 similar 法 k_gain 已经在 x/y 里了, 直接用; pnp 法要再乘 goal_gain
                 (原代码写死 ×2), z 两个方法都不放大。
    quat         pnp 法: 相机在 tag 世界系下的姿态 (x,y,z,w); similar 法没有姿态, None。
    """
    tag_id: int
    method: str
    corners: np.ndarray                  # (4,2) 图像角点, 已按锁定的角点顺序
    x: float
    y: float
    z: float
    goal_mm: Tuple[float, float, float]
    quat: Optional[np.ndarray] = None    # 仅 pnp
    R_CT: Optional[np.ndarray] = None    # 仅 pnp, 画坐标轴用
    tvec: Optional[np.ndarray] = None
    R_TW: Optional[np.ndarray] = None

    @property
    def position_mm(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def describe(self) -> str:
        q = '' if self.quat is None else f' 姿态四元数[{self.quat[0]:.2f} {self.quat[1]:.2f} ' \
                                         f'{self.quat[2]:.2f} {self.quat[3]:.2f}]'
        return (f'id={self.tag_id} {self.method} 位置({self.x:.1f}, {self.y:.1f}, '
                f'{self.z:.1f})mm{q}')


# ============================================================
# 识别 + 位姿
# ============================================================

class TagTracker:
    """认 tag 算位姿的那套东西 (检测器 / 内参 / 世界系 / 卡尔曼), **建一次反复用**。

    有状态的三样, 都在这一个实例里, 节点不用自己拿:
        _corner_shift  solvePnP 的角点顺序。第一次认到时自动试 4 种顺序, 挑一个
                       "tag 左上角在图像左边"的锁住 —— 不锁的话四元数会隔帧翻一下。
        _R_TW          世界坐标系。第一次认到 tag 时锁在 tag 上, 之后不再变。
        _kalman        X/Y 的卡尔曼滤波器, 第一次拿到有效 z 之后初始化。
    """

    def __init__(self, params: Optional[AprilTagParams] = None,
                 intrinsics: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                 **overrides):
        p = default_params()
        if params is not None:
            p = params
        if overrides:
            p = replace(p, **overrides)      # 名字写错直接 TypeError, 不静默忽略
        self.params = p.resolved()

        # 内参: 正常从 calib_file 读; 自检时直接塞一份假的进来 (不用真标定文件)
        if intrinsics is None:
            intrinsics = load_intrinsics(self.params.calib_file)
        self.inmtx, self.distortion = intrinsics
        self.fx = float(self.inmtx[0, 0])
        self.fy = float(self.inmtx[1, 1])
        self.cx = float(self.inmtx[0, 2])
        self.cy = float(self.inmtx[1, 2])

        self.detector = self._make_detector()

        # ---- 有状态的东西 ----
        self._corner_shift: Optional[int] = None
        self._R_TW: Optional[np.ndarray] = None
        self._kalman: Optional[cv2.KalmanFilter] = None
        self._last_measurement: Optional[np.ndarray] = None
        self._last_t: Optional[float] = None

    # ------------------ 检测 ------------------

    def _make_detector(self):
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, DICT_NAME))
        dp = cv2.aruco.DetectorParameters()
        if self.params.corner_refine and hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
            dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return cv2.aruco.ArucoDetector(dictionary, dp)

    def detect(self, frame: np.ndarray) -> List[TagCorners]:
        """一帧里所有 tag (BGR 图进去, 不挑 id, 不排序)。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        out = []
        if ids is None:
            return out
        for i in range(len(ids)):
            tid = int(np.asarray(ids[i]).reshape(-1)[0])
            out.append(TagCorners(tag_id=tid, corners=corners[i].reshape(-1, 2)))
        return out

    def pick(self, tags: Sequence[TagCorners]) -> Optional[TagCorners]:
        """按 params.tag_id 挑一个: 指定了 id 就认那个 (认到就立刻返回), 否则认最大的。

        多个 tag 同时在画面里是常态 (场地边上一圈 tag) —— 大的是近的, 近的是我们要的。
        """
        if not tags:
            return None
        if not self.params.wants_largest:
            want = int(self.params.tag_id)
            for t in tags:
                if t.tag_id == want:
                    return t
        return max(tags, key=lambda t: t.area)

    # ------------------ 对外: 一步到位 ------------------

    def track(self, frame: np.ndarray) -> Optional[TagPose]:
        """认 + 挑 + 算位姿。认到返回 TagPose, 没认到返回 None (没 tag 是常态)。"""
        tag = self.pick(self.detect(frame))
        if tag is None:
            return None
        if self.params.method == 'similar':
            return self._pose_similar(tag)
        return self._pose_pnp(tag)

    # ------------------ 相似三角形 (原 cam_pos) ------------------

    def _pose_similar(self, tag: TagCorners) -> Optional[TagPose]:
        """相似三角形法: 由 tag 的像素边长/质心估计位置 (mm)。

            Z = f * S / s     f=焦距(像素) S=tag 真实边长 s=像素边长
            X/Y 由质心偏离光轴的偏移反算
        只用 fx/fy/cx/cy, 不依赖 solvePnP 的旋转解算 —— tag 有点歪的时候距离照样准。
        """
        s = tag.edge_px
        if s < 1e-6:
            return None                      # 退化成一个点, 算不出来
        uc, vc = tag.center
        f_avg = 0.5 * (self.fx + self.fy)
        Z_mm = f_avg * self.params.tag_size_mm / s

        # 图像系 (u 向右, v 向下) 与我们要报的方向不一致, 这里按原来的口径反算
        Y_mm = (uc - self.cx) / self.fx * Z_mm
        X_mm = (vc - self.cy) / self.fy * Z_mm

        k = self.params.k_gain                # 相机不方便挪, 放大一点让响应明显
        X_mm *= k
        Y_mm *= k
        return TagPose(tag_id=tag.tag_id, method='similar', corners=tag.corners,
                       x=float(X_mm), y=float(Y_mm), z=float(Z_mm),
                       goal_mm=(float(X_mm), float(Y_mm), float(Z_mm)))

    # ------------------ PnP 全位姿 (原 Apriltag_pose) ------------------

    def _pose_pnp(self, tag: TagCorners) -> Optional[TagPose]:
        try:
            R_CT, tvec = self._solve_pose(tag.corners)
        except RuntimeError as e:
            # solvePnP 偶发解不出来 (角点太糊), 这一帧当没认到就行, 别把取图线程带走
            self.last_error = str(e)
            return None

        if self._R_TW is None:
            self._R_TW = self._build_world_frame(R_CT)
        R_TW = self._R_TW

        # 相机在 tag 世界系下的位置/姿态
        Camera_w = R_TW.T @ (-R_CT.T @ tvec)
        R_WC = R_TW.T @ R_CT.T
        quat = self._rot_to_quat(R_WC)

        raw_x, raw_y, raw_z = float(Camera_w[0]), float(Camera_w[1]), float(Camera_w[2])
        fx_, fy_, fz_ = self._filter(raw_x, raw_y, raw_z)

        gain = self.params.goal_gain
        return TagPose(tag_id=tag.tag_id, method='pnp', corners=tag.corners,
                       x=fx_, y=fy_, z=fz_,
                       goal_mm=(fx_ * gain, fy_ * gain, fz_),
                       quat=quat, R_CT=R_CT, tvec=tvec, R_TW=R_TW)

    def _filter(self, raw_x, raw_y, raw_z) -> Tuple[float, float, float]:
        """X/Y 过卡尔曼, Z 直接用原始值 (深度上滤波容易把"真的靠近了"滤掉)。"""
        if not self.params.kalman:
            return raw_x, raw_y, raw_z

        now = time.monotonic()
        if self._kalman is None:
            if abs(raw_z) <= 1.0:            # 这个数基本是解歪了, 先别初始化滤波器
                return raw_x, raw_y, raw_z
            self._init_kalman(raw_x, raw_y)
            self._last_t = now
            return raw_x, raw_y, raw_z

        self._kalman.predict()               # 先按匀速模型推一步, 再拿观测修
        last = self._last_measurement[:2].flatten()
        dt = now - self._last_t if self._last_t is not None else 0.0
        if dt > 0:
            vx_meas = (raw_x - last[0]) / dt
            vy_meas = (raw_y - last[1]) / dt
        else:
            vx_meas, vy_meas = 0.0, 0.0
        measurement = np.array([[raw_x], [raw_y], [vx_meas], [vy_meas]], dtype=np.float32)
        self._kalman.correct(measurement)
        state = self._kalman.statePost
        self._last_measurement = measurement.copy()
        self._last_t = now
        return float(state[0, 0]), float(state[1, 0]), raw_z

    def _solve_pose(self, corners) -> Tuple[np.ndarray, np.ndarray]:
        """solvePnP 解 tag 位姿, 返回 (R_CT, tvec)。首次自动锁定角点顺序。"""
        obj_pts = _OBJ_UNIT * self.params.tag_size_mm
        img_pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)

        if self._corner_shift is None:
            # 4 种顺序各试一次, 挑"左上角在最左边"的那个: 就是让 shifted[1].x - shifted[0].x
            # 最大 (所以取负最小)。不锁的话四元数会隔帧翻一下, 看着像在抖。
            best_shift = min(range(4), key=lambda s: -(np.roll(img_pts, -s, axis=0)[1, 0]
                                                       - np.roll(img_pts, -s, axis=0)[0, 0]))
            shifted = np.roll(img_pts, -best_shift, axis=0)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, shifted, self.inmtx, self.distortion, flags=0)
            if not ok:
                raise RuntimeError('solvePnP 失败')
            R, _ = cv2.Rodrigues(rvec)
            self._corner_shift = best_shift
            return R, tvec.reshape(3)

        shifted = np.roll(img_pts, -self._corner_shift, axis=0)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, shifted, self.inmtx, self.distortion, flags=0)
        if not ok:
            raise RuntimeError('solvePnP 失败')
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.reshape(3)

    def _init_kalman(self, x, y):
        kalman = cv2.KalmanFilter(4, 4, 0)
        kalman.transitionMatrix = np.array([
            [1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        kalman.measurementMatrix = np.array([
            [1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.5
        kalman.errorCovPre = np.eye(4, dtype=np.float32) * 100.0
        state = np.array([[float(x)], [float(y)], [0.0], [0.0]], dtype=np.float32)
        kalman.statePre = state.copy()
        kalman.statePost = state.copy()
        self._kalman = kalman
        self._last_measurement = state.copy()

    # ------------------ 世界系 ------------------

    @property
    def world_locked(self) -> bool:
        return self._R_TW is not None

    @property
    def corner_shift(self) -> Optional[int]:
        return self._corner_shift

    def reset(self):
        """把世界系/角点顺序/滤波器全清掉 —— 换个场地、挪了 tag 之后要重来一遍。"""
        self._corner_shift = None
        self._R_TW = None
        self._kalman = None
        self._last_measurement = None
        self._last_t = None

    @staticmethod
    def _build_world_frame(R_CT):
        """把世界系锁在当前这个 tag 上: Z 轴朝上 (tag 平放/立着都认这个), X 轴挑一个
        与相机初始朝向最接近的水平方向当正方向。"""
        X_cam_init = np.array([0, -1, 0], dtype=np.float64)
        X_tag_Init = R_CT.T @ X_cam_init
        X_tag_Init /= np.linalg.norm(X_cam_init)
        candidates = np.array([[1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0],
                               [0.0, -1.0, 0.0],
                               [-1.0, 0.0, 0.0]])
        x_world = candidates[int(np.argmax(candidates @ X_tag_Init))]
        z_world = np.array([0.0, 0.0, 1.0])
        y_world = np.cross(z_world, x_world)
        norm_y = np.linalg.norm(y_world)
        if norm_y < 1e-9:
            raise RuntimeError('X 轴与 Z 轴平行, 构不出 Y 轴。')
        y_world /= norm_y
        return np.column_stack([x_world, y_world, z_world])

    @staticmethod
    def _rot_to_quat(R):
        R = np.asarray(R, dtype=np.float64)
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])

    # ------------------ 调试绘制 ------------------

    def draw_debug(self, frame: np.ndarray, pose: TagPose) -> np.ndarray:
        """在图上画出来 (不改原帧): 相似三角形法画框+距离, pnp 法再画三根世界系坐标轴。"""
        img = frame.copy()
        c = np.asarray(pose.corners, dtype=np.int32)
        cv2.polylines(img, [c], True, (0, 255, 0), 2)
        if pose.method == 'similar':
            centroid = c.mean(axis=0).astype(int)
            cv2.circle(img, tuple(centroid), 5, (0, 255, 255), -1)
            cv2.putText(img, f'{pose.z:.0f} mm', (int(c[0][0]) + 10, int(c[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            return img

        # pnp: 世界系坐标轴 X红 Y绿 Z蓝 + 标出锁定顺序里的左上角
        self._draw_world_axes(img, pose.R_TW, pose.R_CT, pose.tvec)
        shift = self._corner_shift
        tl = c[shift if shift is not None else 0]
        cv2.circle(img, tuple(tl), 8, (0, 255, 255), -1)
        cv2.putText(img, 'top-left', (int(tl[0]) + 15, int(tl[1]) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return img

    def _draw_world_axes(self, img, R_TW, R_CT, tvec):
        """在 tag 中心画世界系坐标轴: X红 Y绿 Z蓝。"""
        length = self.params.axis_len_mm
        thickness = int(self.params.axis_thickness)
        origin_cam = np.asarray(tvec, dtype=np.float64).reshape(3)
        dirs_cam = R_CT @ R_TW
        pts = [origin_cam] + [origin_cam + dirs_cam[:, k] * length for k in range(3)]
        pts = np.float32(pts).reshape(-1, 1, 3)
        imgpts, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3),
                                      self.inmtx, self.distortion)
        imgpts = imgpts.reshape(-1, 2).astype(int)
        o = tuple(imgpts[0])
        cv2.line(img, o, tuple(imgpts[1]), (0, 0, 255), thickness)
        cv2.line(img, o, tuple(imgpts[2]), (0, 255, 0), thickness)
        cv2.line(img, o, tuple(imgpts[3]), (255, 0, 0), thickness)
        return img


# ============================================================
# 合成图 (use_sim 用; 原 Apriltag_image_pub 那块)
# ============================================================

def make_tag_image(width, height, tag_id=DEFAULT_TAG_ID, margin=8):
    """合成一帧"画面正中一个 AprilTag"的图, 尺寸就是相机分辨率。"""
    marker_px = int(min(320, min(width, height) - margin))
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
    frame = np.full((height, width, 3), 200, dtype=np.uint8)
    x0 = width // 2 - marker_px // 2
    y0 = height // 2 - marker_px // 2
    frame[y0:y0 + marker_px, x0:x0 + marker_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


# ============================================================
# 自检: python3 apriltag.py (合成 tag 图 + 假内参, 不碰相机也不用 ROS)
# ============================================================

_FAKE_INMTX = np.array([[400.0, 0.0, 320.0],
                        [0.0, 400.0, 240.0],
                        [0.0, 0.0, 1.0]], dtype=np.float64)
_FAKE_DIST = np.zeros(5, dtype=np.float64)


def _paste(frame, tag_id, marker_px, cx, cy, margin=8):
    """往画布上贴一个 tag (自检用, 想摆哪儿摆哪儿)。"""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
    x0, y0 = int(cx - marker_px / 2), int(cy - marker_px / 2)
    frame[y0:y0 + marker_px, x0:x0 + marker_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


def _selftest() -> bool:
    ok = True

    def check(name, cond, extra=''):
        nonlocal ok
        ok = ok and bool(cond)
        print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')

    # ---- 标定文件: 模块自己找得到吗 ----
    calib = default_calib_file()
    check('自动找得到标定文件', bool(calib) and calib.endswith(CALIB_NAME), calib or '→ 没找到')
    if calib:
        inmtx, dist = load_intrinsics(calib)
        check('读出来的内参是主相机那只', abs(inmtx[0, 0] - 442.78) < 1.0,
              f'fx={inmtx[0, 0]:.2f} 畸变{len(dist)}项')
    check('calib_file 留空会被展开成实际路径',
          default_params().resolved().calib_file != '')
    check('method 写错会当场报错', _raises(lambda: default_params(method='pnp2').resolved()))
    check('tag_size 写 0 会当场报错', _raises(lambda: default_params(tag_size_mm=0).resolved()))

    # ---- 合成图 + 假内参: 不用真相机 ----
    fake = (_FAKE_INMTX, _FAKE_DIST)
    p = default_params(tag_size_mm=40.0, tag_id=1, calib_file=calib or 'x.json')
    tr = TagTracker(p, intrinsics=fake)
    check('参数展开成实际值', tr.params.method == 'pnp' and tr.params.tag_size_mm == 40.0,
          tr.params.describe())

    frame = make_tag_image(640, 480, tag_id=1)
    tags = tr.detect(frame)
    check('认到 1 个 tag', len(tags) == 1, tags[0].describe() if tags else '→ 一个都没有')
    check('id 对得上', bool(tags) and tags[0].tag_id == 1)
    check('四次角点齐', bool(tags) and tags[0].corners.shape == (4, 2))

    # blank / 噪声: 不该认出东西
    blank = np.full((480, 640, 3), 200, np.uint8)
    check('空图不报 tag', tr.track(blank) is None)
    rng = np.random.default_rng(0)
    check('噪声图不报 tag',
          tr.track(rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)) is None)

    # ---- 相似三角形法 ----
    ts = TagTracker(default_params(method='similar', k_gain=0.0), intrinsics=fake)
    pose = ts.track(frame)
    check('similar 算出位姿', pose is not None, pose.describe() if pose else '→ None')
    if pose is not None:
        check('摆正中的 tag: X/Y 约等于 0', abs(pose.x) < 3 and abs(pose.y) < 3,
              f'({pose.x:.2f}, {pose.y:.2f})')
        s = tags[0].edge_px
        expect = 400.0 * 40.0 / s
        check('Z = f*S/s 对得上', abs(pose.z - expect) < 1e-6, f'{pose.z:.1f}mm')
        check('similar 不给姿态', pose.quat is None)
        # 靠得更近 (tag 画得更大) 距离应该更小。
        # 贴图别贴到边 —— 四周要留白 (quiet zone), 顶到边就认不出来了。
        near = TagTracker(default_params(method='similar', k_gain=0.0), intrinsics=fake)
        p_near = near.track(_paste(np.full((480, 640, 3), 200, np.uint8), 1, 400,
                                   320, 240))
        check('tag 更大 → 距离更近',
              p_near is not None and p_near.z < pose.z,
              f'{pose.z:.0f}mm → {p_near.z:.0f}mm' if p_near else '→ None')
        # tag_size 翻倍 → 距离翻倍
        big = TagTracker(default_params(method='similar', k_gain=0.0, tag_size_mm=80.0),
                         intrinsics=fake)
        check('tag_size 翻倍 → Z 翻倍', abs(big.track(frame).z - 2 * pose.z) < 1e-6)

    # ---- PnP 全位姿 ----
    tp = TagTracker(default_params(method='pnp', kalman=False), intrinsics=fake)
    pf = tp.track(frame)
    check('pnp 算出位姿', pf is not None, pf.describe() if pf else '→ None')
    if pf is not None:
        R = pf.R_CT
        check('R 是正交的', np.allclose(R @ R.T, np.eye(3), atol=1e-6),
              f'行列式={np.linalg.det(R):.4f}')
        check('tag 在相机正前方', pf.tvec is not None and pf.tvec[2] > 0,
              f'z={pf.tvec[2]:.1f}mm' if pf.tvec is not None else '')
        check('角点顺序锁定了', tp.corner_shift is not None, f'shift={tp.corner_shift}')
        check('世界系锁定了', tp.world_locked)
        check('四元数是单位长度',
              pf.quat is not None and abs(np.linalg.norm(pf.quat) - 1.0) < 1e-6)
        check('goal 的 x/y 乘了 goal_gain',
              abs(pf.goal_mm[0] - pf.x * 2.0) < 1e-9 and abs(pf.goal_mm[2] - pf.z) < 1e-9)
        # 第二帧: 世界系不该重建, 角点顺序不该再试
        R_TW_before = tp._R_TW.copy()
        pf2 = tp.track(frame)
        check('第二帧沿用同一个世界系', pf2 is not None
              and np.allclose(tp._R_TW, R_TW_before), '' if pf2 else '→ None')
        check('第二帧位置基本不变', pf2 is not None and abs(pf2.z - pf.z) < 5.0,
              f'{pf.z:.1f} → {pf2.z:.1f}mm' if pf2 else '')
        tp.reset()
        check('reset 之后世界系清空', not tp.world_locked and tp.corner_shift is None)

        # kalman 开起来也不该炸, 且第一帧给原始值
        tk = TagTracker(default_params(method='pnp', kalman=True), intrinsics=fake)
        k1 = tk.track(frame)
        k2 = tk.track(frame)
        check('卡尔曼开启后两帧都出得来', k1 is not None and k2 is not None)
        check('卡尔曼第一帧不过滤 (给原始值)',
              k1 is not None and abs(k1.x - pf.x) < 1e-6)

    # ---- 多个 tag: 认哪个 ----
    two = np.full((480, 640, 3), 200, np.uint8)
    _paste(two, 1, 240, 160, 240)      # 大的 (近) 在左
    _paste(two, 2, 120, 480, 240)      # 小的 (远) 在右
    t2 = TagTracker(default_params(tag_id=-1, kalman=False), intrinsics=fake)
    got = t2.pick(t2.detect(two))
    check('tag_id<0 时认最大的那个', got is not None and got.tag_id == 1,
          got.describe() if got else '→ None')
    t3 = TagTracker(default_params(tag_id=2, kalman=False), intrinsics=fake)
    got3 = t3.pick(t3.detect(two))
    check('指定 id 就认那个', got3 is not None and got3.tag_id == 2,
          got3.describe() if got3 else '→ None')

    # ---- 调试绘制: 别改原帧, 出来一张同样大小的图 ----
    ti = TagTracker(default_params(kalman=False), intrinsics=fake)
    pi = ti.track(frame)
    before = frame.copy()
    img = ti.draw_debug(frame, pi)
    check('draw_debug 不改原帧', np.array_equal(frame, before))
    check('draw_debug 尺寸不变', img.shape == frame.shape)
    img_s = ts.draw_debug(frame, ts.track(frame))
    check('similar 的调试图也画得出来', img_s.shape == frame.shape)

    print('\n' + ('全部通过' if ok else '有失败项'))
    return ok


def _raises(fn) -> bool:
    """fn() 该抛异常 —— 抛了返回 True (参数写错要当场炸, 不能带病往下跑)。"""
    try:
        fn()
    except Exception:
        return True
    return False


if __name__ == '__main__':
    import sys
    sys.exit(0 if _selftest() else 1)
