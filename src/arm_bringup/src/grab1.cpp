// grab1 用于从转盘上抓取物体，需实时跟踪

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stereo_msgs/msg/uint8.hpp>

class Grab1Node : public rclcpp::Node
{
    Grab1Node() : Node("grab_node")
    {
        auto sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "real_joint_states", 10,
            std::bind(&Grab1Node::joint_callback, this, std::placeholders::_1));

        auto pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("grab_position", 10);

        auto server_ = 
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg){

    }


};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Grab1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}