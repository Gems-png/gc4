#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

static constexpr double PI = 3.14159265358979323846;

/**
 * 逆运动学节点：目标点 -> 四轴关节角
 *
 * 输入 /goal_position (PointStamped)   目标点，base_link 下的 mm
 * 输出 /joint_states   (JointState)    已减零位偏置、可直接下发的角度
 *      /ik_target_marker (Marker)      RViz 里看目标点（绿=可达 红=不可达）
 *
 * 没有界面窗口，可视化全在 RViz 看（见 myfk 的 ArmMarker）。
 *
 * 本节点是 myfk 的精确逆（末端保持水平分支），两组几何参数必须一致：
 * 改一个必须改另一个，否则 FK(IK(p)) != p —— 之前 258mm 的误差就是这么来的。
 *
 * 单位：输入 mm。相机系 -> base_link 的换算（含手眼标定结果）不在这里做，
 * 由 material / circle 服务负责，本节点只吃 base_link 下的目标点。
 */
class MyIKNode : public rclcpp::Node
{
public:
  MyIKNode() : Node("my_ik_node")
  {
    // ---------- 几何参数（mm / rad），与 myfk 同名同值 ----------
    this->declare_parameter("l1", 50.0);       // 底座到肩关节的高度
    this->declare_parameter("l2", 135.67);     // 大臂
    this->declare_parameter("l3", 205.23);     // 小臂
    this->declare_parameter("l4", 91.76);      // 末端杆：末端水平时整段落在径向
    this->declare_parameter("bias2", 0.308);   // joint2 实测零位偏置
    this->declare_parameter("bias4", 1.1088);  // joint4 实测零位偏置
    // 肘部镜像分支：两支都能精确闭合，只是肘关节朝上/朝下的装法不同。
    // 上机发现大臂往反方向翻，把这里改成 -1.0 即可，不用改代码。
    this->declare_parameter("elbow_sign", -1.0);

    // ---------- 插值参数 ----------
    // 插值一步 = 发布一次，所以降低 publish_hz 会同时放慢运动速度。
    this->declare_parameter("publish_hz", 666.0);
    this->declare_parameter("max_step_rad", 0.05);
    this->declare_parameter("max_steps", 30);

    l1_ = this->get_parameter("l1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    l3_ = this->get_parameter("l3").as_double();
    l4_ = this->get_parameter("l4").as_double();
    bias2_ = this->get_parameter("bias2").as_double();
    bias4_ = this->get_parameter("bias4").as_double();
    elbow_sign_ = this->get_parameter("elbow_sign").as_double() >= 0.0 ? 1.0 : -1.0;
    const double publish_hz = this->get_parameter("publish_hz").as_double();
    max_step_rad_ = this->get_parameter("max_step_rad").as_double();
    max_steps_ = static_cast<int>(this->get_parameter("max_steps").as_int());

    if (publish_hz <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_hz=%.1f 不合法，按 666Hz 处理", publish_hz);
      publish_period_ = std::chrono::microseconds(1500);
    } else {
      publish_period_ = std::chrono::microseconds(static_cast<int64_t>(1e6 / publish_hz));
    }
    if (max_step_rad_ <= 0.0) {
      max_step_rad_ = 0.05;
    }

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "goal_position", 10, std::bind(&MyIKNode::goal_callback, this, std::placeholders::_1));
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "ik_target_marker", 10);

    RCLCPP_INFO(this->get_logger(),
      "my_ik_node 已启动: l1=%.2f l2=%.2f l3=%.2f l4=%.2f bias2=%.4f bias4=%.4f elbow=%+.0f "
      "| 输入 /goal_position (mm), 输出 /joint_states, 目标点见 /ik_target_marker",
      l1_, l2_, l3_, l4_, bias2_, bias4_, elbow_sign_);

    current_joints_ = {0.0, 0.0, 0.0, 0.0};
    publish_thread_ = std::thread(&MyIKNode::publishLoop, this);
  }

  ~MyIKNode() override {
    running_ = false;
    if (publish_thread_.joinable()) publish_thread_.join();
  }

private:
  /**
   * 逆解：末端保持水平（θ3 = a1 - a2 - a3 = 0）时的解析解。
   *
   * 正运动学（见 myfk_node.cpp，两组公式必须成对修改）：
   *   R = l2·cos(a1) + l3·cos(a1-a2) + l4·cos(a1-a2-a3)
   *   Z = l1 + l2·sin(a1) + l3·sin(a1-a2) + l4·sin(a1-a2-a3)
   * 其中 a1 = q2 + bias2, a2 = q3, a3 = q4 + bias4 都是相对水平面的仰角。
   *
   * 末端水平 => a1-a2-a3 = 0，第 4 根杆整段落在径向（cos0=1, sin0=0）：
   *   R' = R - l4
   *   Z' = Z - l1
   * 于是退化成标准两杆问题，解出 a1、a2 后令 a3 = a1 - a2 把末端掰回水平。
   *
   * 数值验证：把解代回上面的正运动学，误差 ~1e-13 mm（见仓库里的往返测试）。
   */
  bool solve(const double x, const double y, const double z,
             std::array<double, 4>& q, bool& reachable)
  {
    reachable = true;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      RCLCPP_ERROR(this->get_logger(), "目标点含 NaN/inf，丢弃");
      return false;
    }
    if (l2_ < 1e-9 || l3_ < 1e-9) {
      RCLCPP_ERROR(this->get_logger(), "杆长 l2/l3 配置为 0，无法解算");
      return false;
    }

    // 1. 底座转角：目标在水平面上的方位角
    const double q1 = std::atan2(y, x);

    // 2. 折到 R-Z 竖直平面
    const double R = std::hypot(x, y);
    const double R2 = R - l4_;              // 末端水平：先把末端杆的长度减掉
    const double Zp = z - l1_;

    // 3. 两杆可达性：末端杆朝外时最短只有 |l2-l3|，够不到就按边界解
    double D = std::hypot(R2, Zp);
    const double D_min = std::abs(l2_ - l3_) + 1e-9;
    const double D_max = l2_ + l3_;

    if (R2 < 0.0) {
      // 目标离底座轴比末端杆还近，末端又要保持水平朝外 —— 这一支无解，
      // 只能尽量往前伸（底座转角仍指向目标，误差写在日志里）
      reachable = false;
      RCLCPP_WARN(this->get_logger(),
        "目标 (%.1f, %.1f, %.1f) 距底座轴 %.1f mm < l4=%.1f mm，末端水平时够不到",
        x, y, z, R, l4_);
    }
    if (D > D_max) {
      reachable = false;
      RCLCPP_WARN(this->get_logger(),
        "目标 (%.1f, %.1f, %.1f) 超出可达范围 (D=%.1f > l2+l3=%.1f)，按边界解算",
        x, y, z, D, D_max);
      D = D_max;
    } else if (D < D_min) {
      reachable = false;
      RCLCPP_WARN(this->get_logger(),
        "目标 (%.1f, %.1f, %.1f) 太近 (D=%.1f < |l2-l3|=%.1f)，按边界解算",
        x, y, z, D, std::abs(l2_ - l3_));
      D = D_min;
    }

    // 4. 三角形：a2 是两杆夹角，beta 是 D 与大臂的夹角
    const double cos_a2 = std::clamp(
      (D * D - l2_ * l2_ - l3_ * l3_) / (2.0 * l2_ * l3_), -1.0, 1.0);
    const double a2 = elbow_sign_ * std::acos(cos_a2);

    double phi = 0.0;
    double beta = 0.0;
    if (D > 1e-9) {
      phi = std::atan2(Zp, R2);   // 目标方向（相对水平面）
      beta = std::acos(std::clamp(
        (l2_ * l2_ + D * D - l3_ * l3_) / (2.0 * l2_ * D), -1.0, 1.0));
    }
    // D=0 且 l2=l3 的完全折叠解：令 phi=beta=0，上式自然给出竖直折回

    const double a1 = phi + elbow_sign_ * beta;
    const double a3 = a1 - a2;    // 末端水平约束 θ3 = 0

    // 5. 减去实测零位偏置，得到下位机要的角度
    q = {q1, a1 - bias2_, a2, a3 - bias4_};
    return true;
  }

  void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const double x = msg->point.x;
    const double y = msg->point.y;
    const double z = msg->point.z;

    std::array<double, 4> q{};
    bool reachable = false;
    if (!solve(x, y, z, q, reachable)) {
      return;   // 解不出来就什么都不发，宁可不动也不要 NaN 下去
    }

    publishTargetMarker(*msg, reachable);

    {
      std::lock_guard<std::mutex> lock(pub_mutex_);
      start_joints_ = current_joints_;           // 从当前实际位置开始插值
      target_joints_ = q;

      double max_delta = 0.0;
      for (int i = 0; i < 4; ++i) {
        max_delta = std::max(max_delta, std::abs(target_joints_[i] - start_joints_[i]));
      }
      total_steps_ = std::max(1, static_cast<int>(std::ceil(max_delta / max_step_rad_)));
      total_steps_ = std::min(total_steps_, max_steps_);
      step_index_ = 0;
    }

    RCLCPP_INFO(this->get_logger(),
      "goal=(%.1f, %.1f, %.1f) -> joints=[%.3f, %.3f, %.3f, %.3f] rad, steps=%d%s",
      x, y, z, q[0], q[1], q[2], q[3], total_steps_, reachable ? "" : " [不可达]");
  }

  /// RViz 里标出目标点：绿=可达，红=不可达。Marker 自己带单位，这里换成米。
  void publishTargetMarker(const geometry_msgs::msg::PointStamped& msg, bool reachable)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = "base_link";     // 目标点本来就是 base_link 下的，不用转换
    m.ns = "ik_target";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = msg.point.x / 1000.0;
    m.pose.position.y = msg.point.y / 1000.0;
    m.pose.position.z = msg.point.z / 1000.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.03;
    m.color.r = reachable ? 0.1f : 1.0f;
    m.color.g = reachable ? 1.0f : 0.1f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration(0, 0);   // 一直留着，下次同 id 覆盖
    target_marker_pub_->publish(m);
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
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.header.frame_id = "base_link";
        js.name = {"joint1", "joint2", "joint3", "joint4"};
        // solve() 里已经减过 bias，这里直接发，serial_comm 转发即可
        js.position = {current_joints_[0], current_joints_[1], current_joints_[2], current_joints_[3]};
        joint_pub_->publish(js);
      }
      std::this_thread::sleep_for(publish_period_);
    }
  }

  double l1_, l2_, l3_, l4_;
  double bias2_, bias4_;
  double elbow_sign_;
  double max_step_rad_;
  int max_steps_;
  std::chrono::microseconds publish_period_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

  std::thread publish_thread_;
  std::atomic<bool> running_{true};
  mutable std::mutex pub_mutex_;          // 保护插值数据
  std::array<double, 4> current_joints_{};
  std::array<double, 4> target_joints_{};  // 目标关节角
  std::array<double, 4> start_joints_{};   // 插值起始关节角
  int step_index_ = 0;                    // 当前步数
  int total_steps_ = 0;                   // 总步数
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyIKNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception&) {
    // Ctrl+C / SIGTERM 收尾，不吐 traceback
  }
  rclcpp::shutdown();
  return 0;
}
