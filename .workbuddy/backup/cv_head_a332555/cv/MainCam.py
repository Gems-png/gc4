import cv2
import circle

class RawImagePublisher(Node):
    
    def __init__(self):
        super().__init__('raw_image_publisher')

        self.declare_parameter('camera_id', 3)
        self.declare_parameter('freq', 15.0)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        camera_id = self.get_parameter('camera_id').value
        self.freq = self.get_parameter('freq').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开apriltag摄像头 {camera_id}')
            raise RuntimeError('摄像头打开失败')

        # ----- 关键修改：强制使用 MJPG -----
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        # 设置分辨率（注意某些摄像头需要先设置格式再设尺寸）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # 即有缓存，又有实时
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

        # 打印实际参数供调试
        real_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        real_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        real_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'实际摄像头参数: {real_w}x{real_h} @ {real_fps:.2f} fps')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_ = self.create_publisher(Image, '/camera/image_raw', qos)

        self._running = True
        self._thread = threading.Thread(target=self._publish_loop)
        self._thread.daemon = True
        self._thread.start()

        self.get_logger().info(f'原始图像发布节点启动，目标帧率 {self.freq:.2f} Hz')