# gc_ik → 树莓派 迁移指南

对比对象：

- `/home/gems/gc_ik/`           —— 桌面端（Ubuntu 22.04 + ROS 2 Humble + OpenCV **5.0.0**）
- `/home/gems/gc_ik(tree/`      —— **树莓派上跑通的版本**（只有 `src/`，无 build/install/log）

对比方式：排除 `build/`、`install/`、`log/`、`__pycache__/`、`venv/` 后逐文件 `diff`。
结论：**47 个源文件里只有 8 个内容不同**，其余全部一致。真正卡住树莓派的只有 1 类问题（OpenCV API 版本），另外 2 类是设备号和构建依赖，剩下的是必须清理的垃圾文件。

---

## 一、环境对比（先看这个）

| 项 | 桌面端 gc_ik | 树莓派 gc_ik(tree | 影响 |
|---|---|---|---|
| 架构 | x86_64 | **aarch64 (arm64)** | 任何含二进制的东西都不能直接拷 |
| OpenCV | 5.0.0（`~/.local/lib/python3.10`） | **4.5.5 / 4.6.0** | ⚠️ **最大坑，见第二节** |
| `cv2.aruco.ArucoDetector` | ✅ 有 | ❌ **没有** | AprilTag 检测代码必须改写 |
| `cv2.aruco.generateImageMarker` | ✅ 有 | ❌ **没有** | sim 模式会崩 |
| `cv2.aruco.detectMarkers()` 自由函数 | ⚠️ 可用（旧接口） | ✅ 有 | 降级后用它 |
| ROS | Humble (Python 3.10) | 同左（源码 `__pycache__` 全是 `cpython-310`） | 代码无差异 |

> 树莓派端 OpenCV 具体版本请自己确认一次：
> ```bash
> python3 -c "import cv2; print(cv2.__version__); print('ArucoDetector:', hasattr(cv2.aruco,'ArucoDetector')); print('generateImageMarker:', hasattr(cv2.aruco,'generateImageMarker'))"
> ```
> 只要 `ArucoDetector` 是 `False`，第一节的改法就适用。

---

## 二、必须改的文件（不改就跑不起来）

### 1. `src/cv/cv/tag_pose.py` ⭐ 核心

**这是树莓派唯一真正的代码适配点。** OpenCV 4.5.x/4.6.x 没有 `cv2.aruco.ArucoDetector` 这个类（4.7 才引入），只有旧的自由函数 `cv2.aruco.detectMarkers()`。

桌面端写法（树莓派上 `AttributeError: module 'cv2.aruco' has no attribute 'ArucoDetector'`）：

```python
# ---------- 初始化 ----------
self.detector = self._make_detector()

def _make_detector(self):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, params)

# ---------- 检测 ----------
corners, ids, _ = self.detector.detectMarkers(gray)
```

树莓派写法（改成自由函数 + 保留 params）：

```python
# ---------- 初始化 ----------
self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
self.params = cv2.aruco.DetectorParameters()
self.get_logger().info("✓ Aruco 初始化成功")

# ---------- 检测 ----------
corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
```

要点：
- `cv2.aruco.getPredefinedDictionary`、`DICT_APRILTAG_36h11`、`DetectorParameters()`、`CORNER_REFINE_SUBPIX` 在 4.5.5 里**都有**，不用动。
- **只有** `ArucoDetector` 和 `detector.detectMarkers()` 这一对要换。
- 树莓派版里 `_make_detector()` 被留成了死代码、`self.dictionary/self.params` 重复赋值了三次——**建议直接删掉重复块**，别照抄那份乱代码。

顺带：树莓派版还多了一段检测日志（可保留，便于排查）：
```python
if ids is None:
    self.get_logger().warn('⚠️ 未检测到任何 AprilTag！')
else:
    self.get_logger().info(f'✅ 检测到 {len(ids)} 个 tag, IDs: {ids.flatten().tolist()}')
```

---

### 2. `src/cv/cv/cam_pos.py` ⚠️ 树莓派版**漏改了**

这个文件在树莓派版里**仍然用的是 `ArucoDetector`（第 52/144-149/161 行）**——和桌面端一模一样，没被修。

它之所以"没出事"，是因为 `pipeline.launch.py` 根本不启动 `cam_pos`，而 `cam_pos.launch.py` 没被用到。**一旦你哪天跑 `ros2 launch cv cam_pos.launch.py`，它会在树莓派上直接崩。**

改法与 `tag_pose.py` **完全相同**（这两个文件是复制关系）：

```
-        self.detector = self._make_detector()          # 第 52 行
+        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
+        self.params = cv2.aruco.DetectorParameters()

-            corners, ids, _ = self.detector.detectMarkers(gray)     # 第 161 行
+            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
```

---

### 3. 摄像头设备号 4 → 0

桌面端摄像头是 `/dev/video4`，树莓派上是 `/dev/video0`。三个文件都要改：

| 文件 | 位置 | 改动 |
|---|---|---|
| `src/cv/cv/raw_image_pub.py` | 第 16 行 | `self.declare_parameter('camera_id', 3)` → `0` |
| `src/cv/launch/pipeline.launch.py` | 第 55 行 `DeclareLaunchArgument('camera_id', ...)` | `default_value='4'` → `'0'` |
| `src/cv/launch/cam_pos.launch.py` | 第 66 行 同上 | `default_value='4'` → `'0'` |

> 这只是默认值，运行时也能用 `camera_id:=N` 覆盖。迁移到新机器**先 `ls /dev/video*` 确认**，别硬套 0。
> `pipeline.launch.py` 第 12 行的示例注释也顺手改了。

---

### 4. `src/gripper_temp/CMakeLists.txt`

补一行显式依赖声明（和 `plate_topic/CMakeLists.txt` 的正确写法对齐）：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)   # ← 新增
```

`gripper_control_publisher.cpp` 用了 `std_msgs::msg::Bool`。桌面端能编过是因为 `ament_target_dependencies()` 会隐式 `find_package`，但**显式写出来在新机器/新环境下更稳**。

### 5. `src/gripper_temp/package.xml`（建议）

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>   <!-- ← 新增 -->
```

不补也能编过（colcon 不看这个），但 `rosdep install` 会漏装。

---

### 6. `src/serial_comm/serial_comm/serial_publiser.py`

树莓派版把 j1 关节的软限幅**注释掉了**：

```python
# 桌面端
self.latest_joint_positions[1] = min(1.5708, self.latest_joint_positions[1])
# 树莓派版
# self.latest_joint_positions[1] = min(1.5708, self.latest_joint_positions[1])
```

**这不是平台差异，是实机调试时临时放开的。** 迁移时按你的实际机械臂决定——如果树莓派带的是真机、桌面端是仿真，那就跟着树莓派；如果两边都是真机，这只是当时为了绕过限幅的临时改动，记得改回去。

---

## 三、必须删掉、不能带过去的文件

这些是"换台机器就出事"的定时炸弹，**迁移任何项目前先清一遍**：

| 路径 | 问题 |
|---|---|
| `src/cv/cv/venv/` | 整个 venv 是 **aarch64 + Python 3.12** 的，`pyvenv.cfg` 里硬编码 `/home/diode/Desktop/gc_ik/src/cv/cv/venv`；含 916 个 `.pyc`、几十 MB 的 `.so`。**每台机器必须自己重建**，绝对不能跨架构/跨用户拷贝 |
| `src/imu/imu/build/`<br>`src/imu/imu/install/`<br>`src/imu/imu/log/` | 嵌套在源码目录里的 colcon 产物，脚本里硬编码 `/home/gems/...`。会污染 colcon 的包发现，且 source 后指向错误路径 |
| 所有 `__pycache__/` | 里面是 `cpython-310` 等旧字节码，跨 Python 小版本会报 `bad magic number` |
| `src/cv/cv/tag_pose_backup.py` | 树莓派上留下的备份文件，**不属于包内容**（`setup.py` 只装 `cv/*.py`，它不会被安装，留着纯干扰） |
| 顶层 `build/ install/ log/` | 同上，且 `install/` 里的 launch/脚本全是本机绝对路径 |

清理命令（在项目根目录执行）：

```bash
rm -rf build install log
rm -rf src/cv/cv/venv
find src -name __pycache__ -type d -exec rm -rf {} +
find src -name "*.pyc" -delete
rm -rf src/imu/imu/build src/imu/imu/install src/imu/imu/log
rm -f  src/cv/cv/tag_pose_backup.py
```

---

## 四、隐性坑（两个版本都一样，但到树莓派上会暴露）

这几处 `diff` 看不出来，但**迁移到树莓派时是高概率踩雷点**：

### 1. `tag_image_pub.py` 用了树莓派没有的 API

`src/cv/cv/tag_image_pub.py:38`：

```python
marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
```

`generateImageMarker` 在 OpenCV 4.5.5 的二进制里**不存在**（已核实）。树莓派之所以没暴露，是因为你们跑的是 `use_sim:=false` 真摄像头，这个节点没被拉起来。**跑 `use_sim:=true` 时会直接 `AttributeError`。**

旧版等价写法：

```python
marker = cv2.aruco.drawMarker(dictionary, tag_id, marker_px)
```

（`drawMarker` 在 4.5.5 里确认存在；4.7+ 反过来把 `drawMarker` 废弃了，所以这段代码在两个版本间无法通用，需要用 `hasattr` 做兼容。）

### 2. 节点启动时创建 tkinter 窗口 → 无显示环境必崩

`tag_pose.py:95` 和 `cam_pos.py:69` 在 `__init__` 里直接：

```python
self.root = tk.Tk()
```

还有 `root.update()` 在主循环里（`tag_pose.py:468`、`cam_pos.py:305`）。

- 没有 `python3-tk` → `ModuleNotFoundError: No module named 'tkinter'`
- 有 tkinter 但 SSH 无显示 → `TclError: no display name and no $DISPLAY environment variable`

树莓派上能跑，说明你们是**在树莓派的桌面环境里直接开终端跑的**（venv 路径 `.../Desktop/gc_ik/...` 也印证了有桌面）。**一旦改成 SSH / 开机自启 / 无头模式，这个节点会立刻崩。**

建议包一层：

```python
try:
    self.root = tk.Tk()
    ...
except Exception as e:
    self.get_logger().warn(f'无显示环境，跳过 GUI: {e}')
    self.root = None
```

并在循环里 `if self.root: self.root.update()`。其他项目迁移时**优先搜 `tkinter` / `PyQt` / `cv2.imshow`**。

### 3. 硬编码的绝对路径

| 文件 | 内容 |
|---|---|
| `src/imu/imu/imu.py:4` | 注释里 `/home/gems/gc_ik/IMU_HI12_Protocol.md` |
| `src/imu/readme.md:28` | `sudo bash /home/gems/gc_ik/src/imu/bind_usb.sh` |
| `.vscode/settings.json` | `"cmake.sourceDirectory": "/home/gems/gc_ik/src/gripper_temp"` |

这些不影响运行（都是注释/文档/编辑器配置），但换用户名就得改。**迁移时统一搜一遍 `/home/`。**

### 4. 相机采集没有强制 V4L2

`raw_image_pub.py:28` 直接 `cv2.VideoCapture(camera_id)`。树莓派上如果接 CSI 摄像头（libcamera），OpenCV 默认后端抓不到，需要 `cv2.VideoCapture(camera_id, cv2.CAP_V4L2)` 或改用 `picamera2`。当前代码适配的是 USB 摄像头。

### 5. `cv/package.xml` 有遗留依赖

```xml
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>opencv-python</depend>
```

`cv_bridge` 和 `image_transport` 已经不用了（readme 里明确写了"去掉 cv_bridge"），`opencv-python` 也不是合法的 rosdep key。新机器上 `rosdep install` 会因此报错或白装一堆东西。**建议删掉这三行。**

### 6. 算力提醒

桌面端 readme 里记录了：`raw_image_pub` 的 `cap.read()` 是**用户态忙等**，实测吃满一个核（CPU 101%）、实际只有 ~4Hz。树莓派 CPU 更弱，这个节点的阻塞式读帧会更明显。迁移后如果延迟大，先怀疑这里，不要怀疑 tag 检测（实测只有 2.3ms）。

---

## 五、迁移 Checklist（可直接套用到其他项目）

按这个顺序过一遍，绝大多数 ROS 2 项目迁到树莓派都能覆盖：

```bash
# 0. 从树莓派拷回来/拷过去之前，先在源目录清理
rm -rf build install log
rm -rf src/*/venv src/*/*/venv
find . -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
find . -name "*.pyc" -delete
rm -rf src/*/build src/*/install src/*/log      # 嵌套的 colcon 产物

# 1. 找硬编码路径
grep -rn "/home/" --include="*.py" --include="*.sh" --include="*.json" \
     --include="*.md" --include="*.txt" --include="*.xml" . | grep -v venv

# 2. 找平台相关二进制（venv / .so / node_modules / 预编译库）
find . -name "*.so" -o -name "*.node" -o -name "venv" -type d | head

# 3. 找 GUI 依赖（无头环境会崩）
grep -rn "tkinter\|PyQt\|PySide\|cv2.imshow\|QApplication" --include="*.py" .

# 4. 找 OpenCV 新 API（树莓派 4.5/4.6 没有）
grep -rn "ArucoDetector\|generateImageMarker\|estimatePoseSingleMarkers" --include="*.py" .

# 5. 到树莓派上确认 OpenCV 能力
python3 -c "import cv2; print(cv2.__version__)"
python3 -c "import cv2; print(hasattr(cv2.aruco,'ArucoDetector'), hasattr(cv2.aruco,'generateImageMarker'))"

# 6. 确认设备号
ls /dev/video*            # 摄像头
ls /dev/ttyUSB*           # 串口
ls -l /dev/imu_usb        # udev 映射（先跑 bind_usb.sh）

# 7. 干净构建（symlink-install 方便反复改）
colcon build --symlink-install
source install/setup.bash
```

**树莓派首次部署额外需要：**

```bash
# 串口权限（否则打不开 /dev/ttyUSB0）
sudo usermod -aG dialout $USER     # 重新登录生效

# IMU 的 udev 映射
sudo cp src/imu/imu_usb.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger

# GUI 依赖（只有保留 tkinter 窗口时才需要）
sudo apt install python3-tk
```

---

## 六、改动一览（速查表）

| # | 文件 | 桌面端 | 树莓派端 | 性质 |
|---|---|---|---|---|
| 1 | `src/cv/cv/tag_pose.py` | `ArucoDetector` 新 API | `detectMarkers()` 自由函数 | ⭐ **必须** |
| 2 | `src/cv/cv/cam_pos.py` | `ArucoDetector` 新 API | **未改，会崩** | ⭐ **必须** |
| 3 | `src/cv/cv/raw_image_pub.py` | `camera_id=3` | `camera_id=0` | ⭐ 设备号 |
| 4 | `src/cv/launch/pipeline.launch.py` | `camera_id='4'` | `'0'` | ⭐ 设备号 |
| 5 | `src/cv/launch/cam_pos.launch.py` | `camera_id='4'` | `'0'` | ⭐ 设备号 |
| 6 | `src/gripper_temp/CMakeLists.txt` | 缺 `find_package(std_msgs)` | 已补 | 构建 |
| 7 | `src/gripper_temp/package.xml` | 缺 `<depend>std_msgs</depend>` | **仍未补** | 建议 |
| 8 | `src/serial_comm/.../serial_publiser.py` | j1 限幅生效 | 注释掉了 | 实机调试 |
| — | `src/cv/cv/tag_pose_backup.py` | 无 | 有 | 垃圾，删 |
| — | `src/cv/cv/venv/` | 无 | aarch64 py3.12 venv | 垃圾，删 |

**其余 39 个文件（imu 全部、myik 全部、plate_topic、serial_comm 其余、cv 的 config/setup.py/package.xml/测试）完全一致，不用动。**

---

## 附：一句话总结

树莓派适配的**唯一硬骨头是 OpenCV 版本 API 差异**（`ArucoDetector` → `detectMarkers`，涉及 `tag_pose.py`、`cam_pos.py`、`tag_image_pub.py` 三个文件），其余全是设备号默认值、依赖声明和必须清理的构建垃圾。迁移其他项目时，**先查 OpenCV 版本再动手**，能省掉 80% 的调试时间。
