#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>

class GripperBothNode : public rclcpp::Node
{
public:
  GripperBothNode() : Node("gripper_both_node")
  {
    // 1. 创建发布者（发送指令）
    cmd_publisher_ = this->create_publisher<std_msgs::msg::Bool>("gripper_cmd", 10);

    // 2. 创建订阅者（接收指令，包括自己发的）
    cmd_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "gripper_cmd", 1,
      std::bind(&GripperBothNode::cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "节点已启动，等待指令...");
    RCLCPP_INFO(this->get_logger(), "输入 1 闭合夹爪，输入 0 张开夹爪，输入 q 退出");
  }

  // 交互式发送指令
  void sendCmd()
  {
    std::cout << "请输入指令 (1/0/q): ";
    std::string input;
    std::cin >> input;

    if (input == "q" || input == "Q") {
      rclcpp::shutdown();   // 触发退出
      return;
    }

    bool isClose;   // true=闭合, false=张开
    if (input == "1") {
      isClose = true;
    } else if (input == "0") {
      isClose = false;
    } else {
      std::cout << "无效输入，请输入 1 或 0" << std::endl;
      return;
    }

    auto msg = std_msgs::msg::Bool();
    msg.data = isClose;
    cmd_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "已发布指令: %d", msg.data);
  }

private:
  void cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到指令 (回调): %d", msg->data);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperBothNode>();

  // 使用 spin_some 循环处理回调，同时主线程负责交互
  rclcpp::Rate rate(10);  // 10Hz，避免空转
  while (rclcpp::ok()) {
    node->sendCmd();                // 阻塞等待用户输入
    rclcpp::spin_some(node);        // 处理积压的回调
    rate.sleep();                   // 让出CPU
  }

  rclcpp::shutdown();
  return 0;
}