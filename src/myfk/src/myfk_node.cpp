#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

static constexpr double PI = 3.14159265358979323846;

/**
 * 正运动学节点：关节角 -> 末端位姿，并在 RViz 里画出机械臂
 *
 * 输入 joint_topic（默认 /real_joint_states，下位机回传的实际角度）
 *      想干跑看模型对不对，可以 -p joint_topic:=/joint_states 听 IK 的指令
 *
 * 输出 fk_position     (PointStamped) 末端点位置，毫米
 *      fk_pose         (PoseStamped)  末端（工具）位姿，毫米 —— 手眼标定的输入
 *      fk_camera_pose  (PoseStamped)  相机位姿 = 末端沿工具的"上"方向抬 camera_offset_mm，毫米
 *      fk_arm_marker   (MarkerArray)  RViz 里的机械臂（连杆+关节+相机位置）
 *
 * 单位说明：机器人这条链上**全是毫米**（杆长、/goal_position、这里的位姿），
 * 和视觉链的 mm 保持一致。只有 RViz 的 Marker 必须是米（REP-103 规定），
 * 换算集中在 publishArmMarker() 一处，别的地方不要出现米。
 *
 * 几何模型（4 根杆都在同一个竖直平面内，再绕 z 轴转 q1）：
 *
 *      z
 *      |      l3        l4
 *      |    /----->  /
 *      |   /        /
 *   l2 |  /        /
 *      | /        /
 *      |/________/__________ x
 *      l1
 *
 *   a1 = q2 + bias2     大臂仰角（相对水平面）
 *   a2 = q3             肘关节角
 *   a3 = q4 + bias4     腕关节角
 *   第3杆仰角 θ2 = a1 - a2
 *   第4杆仰角 θ3 = a1 - a2 - a3   （末端水平的抓取姿态下 θ3 = 0）
 *
 *   R = l2·cos(a1) + l3·cos(θ2) + l4·cos(θ3)
 *   Z = l1 + l2·sin(a1) + l3·sin(θ2) + l4·sin(θ3)
 *   X = cos(q1)·R,  Y = sin(q1)·R
 *
 * myik 里的逆解就是这个式子的反解（末端水平分支），两组参数必须保持一致，
 * 改一个必须改另一个，否则 FK(IK(p)) != p。
 */
class MyFkNode : public rclcpp::Node
{
public:
  MyFkNode() : Node("myfk_node")
  {
    // ---------- 几何参数（mm / rad），与 myik 同名同值 ----------
    this->declare_parameter("l1", 50.0);
    this->declare_parameter("l2", 135.67);
    this->declare_parameter("l3", 205.23);
    this->declare_parameter("l4", 91.76);
    this->declare_parameter("bias2", 0.308);   // joint2 实测零位偏置
    this->declare_parameter("bias4", 1.1088);  // joint4 实测零位偏置

    this->declare_parameter("joint_topic", "/real_joint_states");
    this->declare_parameter("publish_hz", 50.0);   // 输出节流：IK 会以几百 Hz 发关节角

    // 末端工具的安装偏角（度）：0 = 工具轴与第 4 根杆同向。
    // 抓取姿态下第 4 杆水平朝外，若实际夹爪是竖直朝下的，这里填 -90。
    // 这个约定上机必须确认一次，它决定 HECal 标出来的东西对不对。
    this->declare_parameter("tool_mount_deg", 0.0);

    // 相机装在末端上方多少毫米（沿工具的"上"方向）。
    // 现在暂定 50（末端上方 5cm），做完手眼标定后按标定结果改。
    this->declare_parameter("camera_offset_mm", 50.0);


    l1_ = this->get_parameter("l1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    l3_ = this->get_parameter("l3").as_double();
    l4_ = this->get_parameter("l4").as_double();
    bias2_ = this->get_parameter("bias2").as_double();
    bias4_ = this->get_parameter("bias4").as_double();
    tool_mount_ = this->get_parameter("tool_mount_deg").as_double() * PI / 180.0;
    camera_offset_ = this->get_parameter("camera_offset_mm").as_double();

    const std::string joint_topic = this->get_parameter("joint_topic").as_string();
    const double publish_hz = this->get_parameter("publish_hz").as_double();
    min_period_ = (publish_hz > 0.0)
      ? std::chrono::duration<double>(1.0 / publish_hz)
      : std::chrono::duration<double>(0.0);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, 10, std::bind(&MyFkNode::jointCallback, this, std::placeholders::_1));

    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("fk_position", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fk_pose", 10);
    camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fk_camera_pose", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fk_arm_marker", 10);

    RCLCPP_INFO(this->get_logger(),
      "myfk_node 已启动: 听 %s, l1=%.2f l2=%.2f l3=%.2f l4=%.2f bias2=%.4f bias4=%.4f "
      "tool_mount=%.1f° 相机=末端上方%.0fmm | 位姿话题单位是毫米, RViz 固定坐标系用 base_link",
      joint_topic.c_str(), l1_, l2_, l3_, l4_, bias2_, bias4_,
      tool_mount_ * 180.0 / PI, camera_offset_);
  }

private:
  /// 一个方向向量：绕 z 轴转 q1 的竖直平面内，仰角 ang（弧度），水平面上朝径向外
  std::array<double, 3> dir(double q1, double ang) const
  {
    const double c = std::cos(ang);
    return {std::cos(q1) * c, std::sin(q1) * c, std::sin(ang)};
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 4) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "关节数 %zu < 4，忽略", msg->position.size());
      return;
    }

    // 节流
    const auto now = std::chrono::steady_clock::now();
    if (last_publish_.time_since_epoch().count() != 0 && now - last_publish_ < min_period_) {
      return;
    }
    last_publish_ = now;

    const double q1 = msg->position[0];
    const double a1 = msg->position[1] + bias2_;   // 大臂仰角
    const double a2 = msg->position[2];            // 肘
    const double a3 = msg->position[3] + bias4_;   // 腕

    if (!std::isfinite(q1) || !std::isfinite(a1) || !std::isfinite(a2) || !std::isfinite(a3)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "关节角含 NaN/inf，忽略");
      return;
    }

    const double th2 = a1 - a2;         // 第3杆仰角
    const double th3 = th2 - a3;        // 第4杆仰角（末端水平时 = 0）

    // ---- 逐杆累加，顺便就是 RViz 要画的骨架（单位 mm）----
    const auto d1 = dir(q1, a1);
    const auto d2 = dir(q1, th2);
    const auto d3 = dir(q1, th3);
    const std::array<double, 3> shoulder = {0.0, 0.0, l1_};
    std::array<double, 3> elbow, wrist, tip;
    for (int i = 0; i < 3; ++i) {
      elbow[i] = shoulder[i] + l2_ * d1[i];
      wrist[i] = elbow[i] + l3_ * d2[i];
      tip[i] = wrist[i] + l4_ * d3[i];
    }

    // ---- 工具坐标系 ----
    //   z_tool = 工具轴（沿第 4 杆，末端水平时水平朝外，再叠 tool_mount 俯仰）
    //   x_tool = 水平切向，y_tool = z_tool × x_tool（末端水平时正好竖直向上）
    const double t = th3 + tool_mount_;
    const double ct = std::cos(t), st = std::sin(t);
    const std::array<double, 3> z_axis = {std::cos(q1) * ct, std::sin(q1) * ct, st};

    std::array<double, 3> x_axis;
    if (std::abs(ct) > 1e-6) {
      // cross(z_world, z_tool) 归一化 = (-sin q1, cos q1, 0)
      x_axis = {-std::sin(q1), std::cos(q1), 0.0};
    } else {
      // 工具轴竖直时叉乘退化，改用径向做参考
      x_axis = {std::cos(q1), std::sin(q1), 0.0};
    }
    const std::array<double, 3> y_axis = {
      z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
      z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
      z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    };

    const auto quat = quaternionFromAxes(x_axis, y_axis, z_axis);
    const auto stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
      ? this->now() : rclcpp::Time(msg->header.stamp);

    // ---- 末端点位置（毫米）----
    geometry_msgs::msg::PointStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "base_link";
    ps.point.x = tip[0];
    ps.point.y = tip[1];
    ps.point.z = tip[2];
    position_pub_->publish(ps);

    // ---- 末端位姿（毫米）----
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ps.header;
    pose.pose.position = ps.point;
    pose.pose.orientation.x = quat[0];
    pose.pose.orientation.y = quat[1];
    pose.pose.orientation.z = quat[2];
    pose.pose.orientation.w = quat[3];
    pose_pub_->publish(pose);

    // ---- 相机位姿（毫米）----
    // 相机装在末端上方 camera_offset：沿 y_tool 平移，姿态暂按与工具同向，
    // 手眼标定要解的就是这个安装变换，标完把 camera_offset_mm 换成实测值。
    geometry_msgs::msg::PoseStamped cam;
    cam.header = ps.header;
    cam.pose.orientation = pose.pose.orientation;
    cam.pose.position.x = ps.point.x + camera_offset_ * y_axis[0];
    cam.pose.position.y = ps.point.y + camera_offset_ * y_axis[1];
    cam.pose.position.z = ps.point.z + camera_offset_ * y_axis[2];
    camera_pose_pub_->publish(cam);

    publishArmMarker(ps.header, shoulder, elbow, wrist, tip,
                     {cam.pose.position.x, cam.pose.position.y, cam.pose.position.z});

    RCLCPP_DEBUG(this->get_logger(),
      "q=[%.3f %.3f %.3f %.3f] -> (%.1f, %.1f, %.1f) mm, θ3=%.1f°",
      q1, msg->position[1], a2, msg->position[3],
      tip[0], tip[1], tip[2], th3 * 180.0 / PI);
  }

  /// 在 RViz 里画机械臂：底座柱+三根连杆、关节球、相机位置
  void publishArmMarker(const std_msgs::msg::Header& header,
                        const std::array<double, 3>& shoulder,
                        const std::array<double, 3>& elbow,
                        const std::array<double, 3>& wrist,
                        const std::array<double, 3>& tip,
                        const std::array<double, 3>& camera)
  {
    auto toPoint = [](const std::array<double, 3>& p) {
      geometry_msgs::msg::Point out;
      out.x = p[0] / 1000.0;      // mm -> m
      out.y = p[1] / 1000.0;
      out.z = p[2] / 1000.0;
      return out;
    };

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header = header;

    // 连杆：底座柱 -> 肩 -> 肘 -> 腕 -> 末端
    m.ns = "fk_arm";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.012;
    m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.1f; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    m.points.push_back(toPoint({0.0, 0.0, 0.0}));
    m.points.push_back(toPoint(shoulder));
    m.points.push_back(toPoint(elbow));
    m.points.push_back(toPoint(wrist));
    m.points.push_back(toPoint(tip));
    arr.markers.push_back(m);

    // 关节球
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.scale.x = m.scale.y = m.scale.z = 0.03;
    m.color.r = 0.2f; m.color.g = 0.7f; m.color.b = 1.0f;
    m.points.clear();
    m.points.push_back(toPoint(shoulder));
    m.points.push_back(toPoint(elbow));
    m.points.push_back(toPoint(wrist));
    m.points.push_back(toPoint(tip));
    arr.markers.push_back(m);

    // 相机位置
    m.id = 2;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = 0.02;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.2f;
    m.points.clear();
    m.pose.position = toPoint(camera);
    arr.markers.push_back(m);

    marker_pub_->publish(arr);
  }

  /// 由三个正交轴（列向量）构造四元数，Shepperd 法，取最大分量避免除以小数
  static std::array<double, 4> quaternionFromAxes(
    const std::array<double, 3>& xa,
    const std::array<double, 3>& ya,
    const std::array<double, 3>& za)
  {
    const double m00 = xa[0], m10 = xa[1], m20 = xa[2];
    const double m01 = ya[0], m11 = ya[1], m21 = ya[2];
    const double m02 = za[0], m12 = za[1], m22 = za[2];

    double qw, qx, qy, qz;
    const double tr = m00 + m11 + m22;
    if (tr > 0.0) {
      const double s = std::sqrt(tr + 1.0) * 2.0;
      qw = 0.25 * s;
      qx = (m21 - m12) / s;
      qy = (m02 - m20) / s;
      qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
      const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
      qw = (m21 - m12) / s;
      qx = 0.25 * s;
      qy = (m01 + m10) / s;
      qz = (m02 + m20) / s;
    } else if (m11 > m22) {
      const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
      qw = (m02 - m20) / s;
      qx = (m01 + m10) / s;
      qy = 0.25 * s;
      qz = (m12 + m21) / s;
    } else {
      const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
      qw = (m10 - m01) / s;
      qx = (m02 + m20) / s;
      qy = (m12 + m21) / s;
      qz = 0.25 * s;
    }
    return {qx, qy, qz, qw};
  }

  double l1_, l2_, l3_, l4_;
  double bias2_, bias4_;
  double tool_mount_;
  double camera_offset_;      // 米
  std::chrono::duration<double> min_period_{0.0};
  std::chrono::steady_clock::time_point last_publish_{};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyFkNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception&) {
    // Ctrl+C / SIGTERM 收尾，不吐 traceback
  }
  rclcpp::shutdown();
  return 0;
}
