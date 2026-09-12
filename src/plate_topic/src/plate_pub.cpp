#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"   // 头文件全小写，消息类型为 Float32


//ros2 topic pub /plate_cmd std_msgs/msg/float32 "{data: 1}" -1

class platePub : public rclcpp::Node
{
public:
  platePub() : Node("platePub")
  {
    // 1. 创建发布者（反馈状态）
    status_publisher_ = this->create_publisher<std_msgs::msg::Float32>("plate_cmd", 10);

    // 2. 创建订阅者（接收指令）
    cmd_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "plate_cmd", 10,
      std::bind(&platePub::cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "节点已启动，等待指令...");

    // 3. 发布一条初始状态（示例）
    auto msg = std_msgs::msg::Float32();   // 修正语法：正确的作用域符
    msg.data = 0.0f;                       // 使用浮点字面量
    status_publisher_->publish(msg);       // 用下划线变量名，并传入消息对象
  }

private:
  void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float command = msg->data;              // 用 float 接收
    RCLCPP_INFO(this->get_logger(), "收到指令: %f", command); // 打印浮点数用 %f
  }

  // 修正：类型统一为 Float32（与构造函数中创建的一致）
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<platePub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}