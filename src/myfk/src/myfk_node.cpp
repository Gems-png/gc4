#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QObject>
#include <QString>

#include <thread>
#include <atomic>
#include <chrono>

static constexpr double PI = 3.14159265358979323846;

/**
 * 逆运动学节点，同时提供Qt界面显示目标点和关节角
 */
class MyIKNode : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  MyIKNode() : Node("my_ik_node")
  {
    // ---------- 杆长参数 mm----------
    this->declare_parameter("l1", 50.0);
    this->declare_parameter("l2", 150.0);
    this->declare_parameter("l3", 200.0);
    l1_ = this->get_parameter("l1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    l3_ = this->get_parameter("l3").as_double();

    // ---------- 订阅与发布 ----------
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "goal_position", 10,
      std::bind(&MyIKNode::goal_callback, this, std::placeholders::_1));
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    RCLCPP_INFO(this->get_logger(),
      "my_ik_node 已启动: l1=%.1f l2=%.1f l3=%.1f", l1_, l2_, l3_);

    // 初始化UI数据
    current_goal_ = {0.0, 0.0, 0.0};
    current_joints_ = {0.0, 0.0, 0.0, 0.0};
    reachable_ = true;

    // 插值相关
    publish_thread_ = std::thread(&MyIKNode::publishLoop, this);
  }

  // 插值需显式析构线程
  ~MyIKNode() {
    running_ = false;
    if (publish_thread_.joinable()) publish_thread_.join();
  }

  // 供UI读取的接口 (线程安全)
  std::tuple<double, double, double> getGoal() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {current_goal_[0], current_goal_[1], current_goal_[2]};
  }
  std::array<double, 4> getJoints() const
  {
    std::lock_guard<std::mutex> lock(pub_mutex_);  // 改为 pub_mutex_
    return current_joints_;
  }
  bool isReachable() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return reachable_;
  }

private:
  // void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  // {

  //   // 左手坐标系，前x正，左y正
  //   const double x = msg->point.x;
  //   const double y = msg->point.y;
  //   const double z = msg->point.z;

  //   // ---- 逆解计算 ----
  //   // atan可以返回大角度， 如何-135
  //   double j0 = std::atan2(y, x);

  //   // if(x < 0 && y > 0)j0 += PI;
  //   // if(x < 0 && y < 0)j0 -= PI;

  //   const double R = std::hypot(x, y);
  //   const double h = z - l1_;
  //   const double D = std::hypot(R, h);

  //   bool reachable = false;
  //   double j1 = 0.0, j2 = 0.0, j3 = 0.0;

  //   if (std::isfinite(D) && D > 1e-9 && l2_ > 1e-9 && l3_ > 1e-9) {
  //     double c2 = (l2_*l2_ + l3_*l3_ - D*D) / (2.0*l2_*l3_);
  //     double c1 = (l2_*l2_ + D*D - l3_*l3_) / (2.0*l2_*D);
  //     reachable = (D <= l2_ + l3_ + 1e-6) && (std::abs(c1) <= 1.0 + 1e-9) && (std::abs(c2) <= 1.0 + 1e-9);

  //     c1 = std::clamp(c1, -1.0, 1.0);
  //     c2 = std::clamp(c2, -1.0, 1.0);
  //     j2 = std::acos(c2);
  //     j1 = PI / 2.0 + std::atan2(R, h) - std::acos(c1);
  //     j3 = std::max(j1 - j2, -0.7854);
  //     if (j1 - j2 < -0.7854) {
  //       RCLCPP_WARN(this->get_logger(), "关节4 超出限位，提高到 -0.7854 rad");
  //     }
  //   } else {
  //     RCLCPP_WARN(this->get_logger(), "目标 (%.1f, %.1f, %.1f) 退化", x, y, z);
  //   }

  //   if (!reachable) {
  //     RCLCPP_WARN(this->get_logger(),
  //       "目标 (%.1f, %.1f, %.1f) 超出可达范围 (D=%.1f > l2+l3=%.1f), 已按边界求解",
  //       x, y, z, D, l2_ + l3_);
  //   }

  //   // 发布关节状态
  //   sensor_msgs::msg::JointState js;
  //   js.header.stamp = this->now();
  //   js.header.frame_id = msg->header.frame_id;
  //   js.name = {"joint1", "joint2", "joint3", "joint4"};
  //   js.position = {j0, j1, j2, j3};
  //   joint_pub_->publish(js);

  //   // 保存最新数据供UI读取
  //   {
  //     std::lock_guard<std::mutex> lock(data_mutex_);
  //     current_goal_ = {x, y, z};
  //     current_joints_ = {j0, j1, j2, j3};
  //     reachable_ = reachable;
  //   }

  //   RCLCPP_INFO(this->get_logger(),
  //     "goal=(%.1f, %.1f, %.1f) -> joints=[%.3f, %.3f, %.3f, %.3f] rad",
  //     x, y, z, j0, j1, j2, j3);
  // }

  void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const double x = msg->point.x;
    const double y = msg->point.y;
    const double z = msg->point.z;

    double j0 = std::atan2(y, x);
    const double R = std::hypot(x, y);
    const double h = z - l1_;
    const double D = std::hypot(R, h);

    bool reachable = false;
    double j1 = 0.0, j2 = 0.0, j3 = 0.0;

    if (std::isfinite(D) && D > 1e-9 && l2_ > 1e-9 && l3_ > 1e-9) {
        double c2 = (l2_*l2_ + l3_*l3_ - D*D) / (2.0*l2_*l3_);
        double c1 = (l2_*l2_ + D*D - l3_*l3_) / (2.0*l2_*D);
        reachable = (D <= l2_ + l3_ + 1e-6) && (std::abs(c1) <= 1.0 + 1e-9) && (std::abs(c2) <= 1.0 + 1e-9);

        c1 = std::clamp(c1, -1.0, 1.0);
        c2 = std::clamp(c2, -1.0, 1.0);
        j2 = std::acos(c2);
        j1 = PI / 2.0 + std::atan2(R, h) - std::acos(c1);
        j3 = std::max(j1 - j2, -0.7854);
        if (j1 - j2 < -0.7854) {
            RCLCPP_WARN(this->get_logger(), "关节4 超出限位，提高到 -0.7854 rad");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "目标 (%.1f, %.1f, %.1f) 退化", x, y, z);
    }

    if (!reachable) {
        RCLCPP_WARN(this->get_logger(),
            "目标 (%.1f, %.1f, %.1f) 超出可达范围 (D=%.1f > l2+l3=%.1f), 已按边界求解",
            x, y, z, D, l2_ + l3_);
    }

    // 更新插值参数（线程安全）
    {
        std::lock_guard<std::mutex> lock(pub_mutex_);
        start_joints_ = current_joints_;           // 从当前实际位置开始插值
        target_joints_ = {j0, j1, j2, j3};
        // 计算最大角度变化
        double max_delta = 0.0;
        for (int i = 0; i < 4; ++i) {
            double delta = std::abs(target_joints_[i] - start_joints_[i]);
            if (delta > max_delta) max_delta = delta;
        }
        // 动态步数：每步最大变化 0.05 rad，至少1步，最多30步（适应30Hz输入）
        total_steps_ = std::max(1, static_cast<int>(std::ceil(max_delta / 0.05)));
        total_steps_ = std::min(total_steps_, 30);
        step_index_ = 0;
        // 更新目标点及可达标志（供UI）
        {
            std::lock_guard<std::mutex> lock2(data_mutex_);
            current_goal_ = {x, y, z};
            reachable_ = reachable;
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "goal=(%.1f, %.1f, %.1f) -> joints=[%.3f, %.3f, %.3f, %.3f] rad, steps=%d",
        x, y, z, j0, j1, j2, j3, total_steps_);
  }

  void publishLoop() {
      while (running_) {
          {
              std::lock_guard<std::mutex> lock(pub_mutex_);
              if (step_index_ < total_steps_) {
                  double alpha = static_cast<double>(step_index_ + 1) / total_steps_;
                  for (int i = 0; i < 4; ++i) {
                      current_joints_[i] = start_joints_[i] + alpha * (target_joints_[i] - start_joints_[i]);
                  }
                  ++step_index_;
              }
              // 发布当前关节状态
              sensor_msgs::msg::JointState js;
              js.header.stamp = this->now();
              js.header.frame_id = "base_link";
              js.name = {"joint1", "joint2", "joint3", "joint4"};
              js.position = {current_joints_[0], current_joints_[1], current_joints_[2], current_joints_[3]};
              joint_pub_->publish(js);
          }
          std::this_thread::sleep_for(std::chrono::microseconds(1500));  // ~666Hz
      }
  }


  double l1_, l2_, l3_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  mutable std::mutex data_mutex_;
  std::array<double, 3> current_goal_;
  std::array<double, 4> current_joints_;
  bool reachable_;

  // 插值相关
  std::thread publish_thread_;
  std::atomic<bool> running_{true};
  mutable std::mutex pub_mutex_;          // 保护插值数据
  std::array<double, 4> target_joints_;   // 目标关节角
  std::array<double, 4> start_joints_;    // 插值起始关节角
  int step_index_ = 0;                    // 当前步数
  int total_steps_ = 0;                   // 总步数
  const double MAX_STEP_RAD = 0.05;       // 每步最大角度变化（可调）
};




// ---------- Qt 主窗口 ----------
class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  MainWindow(MyIKNode* node, QWidget* parent = nullptr)
    : QMainWindow(parent), node_(node)
  {
    setWindowTitle("IK 监视器");
    resize(300, 250);

    QWidget* central = new QWidget(this);
    setCentralWidget(central);
    QVBoxLayout* layout = new QVBoxLayout(central);

    goal_label_ = new QLabel("目标点: (0.0, 0.0, 0.0)", this);
    joints_label_ = new QLabel("关节角: [0.00, 0.00, 0.00, 0.00] rad", this);
    status_label_ = new QLabel("状态: 可达", this);

    layout->addWidget(goal_label_);
    layout->addWidget(joints_label_);
    layout->addWidget(status_label_);

    // 定时刷新 (100 ms)
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::updateDisplay);
    timer_->start(100);
  }

private slots:
  void updateDisplay()
  {
    if (!node_) return;
    auto goal = node_->getGoal();
    auto joints = node_->getJoints();
    bool ok = node_->isReachable();

    goal_label_->setText(QString("目标点: (%1, %2, %3)")
      .arg(std::get<0>(goal), 0, 'f', 1)
      .arg(std::get<1>(goal), 0, 'f', 1)
      .arg(std::get<2>(goal), 0, 'f', 1));
    joints_label_->setText(QString("关节角: [%1, %2, %3, %4] rad")
      .arg(joints[0], 0, 'f', 3)
      .arg(joints[1], 0, 'f', 3)
      .arg(joints[2], 0, 'f', 3)
      .arg(joints[3], 0, 'f', 3));
    status_label_->setText(ok ? "状态: 可达" : "状态: 不可达 (边界解)");
  }

private:
  MyIKNode* node_;
  QLabel* goal_label_;
  QLabel* joints_label_;
  QLabel* status_label_;
  QTimer* timer_;
};

// ---------- 主函数 ----------
int main(int argc, char** argv)
{
  // 初始化Qt
  QApplication app(argc, argv);

  // 初始化ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyIKNode>();

  // 创建主窗口
  MainWindow window(node.get());
  window.show();

  // 使用Qt定时器轮询ROS2 (10 Hz)
  QTimer ros_timer;
  ros_timer.setInterval(100); // ms
  QObject::connect(&ros_timer, &QTimer::timeout, [&]() {
    rclcpp::spin_some(node);
  });
  ros_timer.start();

  // 进入Qt事件循环
  int ret = app.exec();

  // 清理
  ros_timer.stop();
  rclcpp::shutdown();
  return ret;
}

// 必须包含 moc 生成的文件（如果使用Q_OBJECT）
#include "my_ik_node.moc"