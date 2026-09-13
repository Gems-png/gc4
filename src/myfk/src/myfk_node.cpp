#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

double BIAS2 = 0.0; // 关节2偏置角度，单位弧度, 等待测量

class MyFKNode : public rclcpp::Node
{
public:
  double l1_, l2_, l3_; // 杆长参数 mm

  MyFKNode() : Node("my_fk_node")
  {
    // ---------- 杆长参数 mm----------
    this->declare_parameter("l1", 50.0);
    this->declare_parameter("l2", 150.0);
    this->declare_parameter("l3", 200.0);

    l1_ = this->get_parameter("l1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    l3_ = this->get_parameter("l3").as_double();

    auto sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&MyFKNode::joint_callback, this, std::placeholders::_1));

    auto pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("fk_position", 10);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 4) {
      RCLCPP_WARN(this->get_logger(), "关节状态消息长度不足4，忽略");
      return;
    }

    double j0 = msg->position[0];
    double j1 = msg->position[1];
    double j2 = msg->position[2] + BIAS2; // 加上偏置角度
    double j3 = msg->position[3];

    // ---- 正解计算 ----
    double x = std::cos(j0) * (this->l2_ * std::cos(j1) + this->l3_ * std::cos(j1 - j2));
    double y = std::sin(j0) * (this->l2_ * std::cos(j1) + this->l3_ * std::cos(j1 - j2));
    double z = this->l1_ + this->l2_ * std::sin(j1) + this->l3_ * std::sin(j1 - j2);

    RCLCPP_INFO(this->get_logger(),
      "正解: j=[%.3f, %.3f, %.3f, %.3f] -> pos=(%.1f, %.1f, %.1f)",
      j0, j1, j2, j3, x, y, z);
  }

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyFKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }