#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

double BIAS2 = 0.308; // 关节2偏置角度，单位弧度, 等待测量
double BIAS4 = 1.1088; // 关节4偏置角度，单位弧度, 等待测量

class MyFKNode : public rclcpp::Node
{
public:
  // 正运动结算，额外算了末端的长度
  double l1_, l2_, l3_, l4_; // 杆长参数 mm

  MyFKNode() : Node("my_fk_node")
  {
    // ---------- 杆长参数 mm----------
    this->declare_parameter("l1", 50.0);
    this->declare_parameter("l2", 135.67);
    this->declare_parameter("l3", 205.23);
    this->declare_parameter("l4", 91.76);

    l1_ = this->get_parameter("l1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    l3_ = this->get_parameter("l3").as_double();
    l4_ = this->get_parameter("l4").as_double();

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
    double j1 = msg->position[1] + BIAS2; // 加上偏置角度
    double j2 = msg->position[2];
    double j3 = msg->position[3] + BIAS4; // 加上偏置角度

    // ---- 正解计算 ----
    tf2::Vector3 position;
    position.setX(std::cos(j0) * (this->l2_ * std::cos(j1) + this->l3_ * std::cos(j1 - j2) + this->l4_ * std::cos(j1 - j2 - j3)));
    position.setY(std::sin(j0) * (this->l2_ * std::cos(j1) + this->l3_ * std::cos(j1 - j2) + this->l4_ * std::cos(j1 - j2 - j3)));
    position.setZ(this->l1_ + this->l2_ * std::sin(j1) + this->l3_ * std::sin(j1 - j2) + this->l4_ * std::sin(j1 - j2 - j3));

    RCLCPP_INFO(this->get_logger(),
      "正解: j=[%.3f, %.3f, %.3f, %.3f] -> pos=(%.1f, %.1f, %.1f)",
      j0, j1, j2, j3, position.x(), position.y(), position.z());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyFKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}