// grab1 用于从转盘上抓取物体，需实时跟踪


// 数据链路2的2.0
// grab1
// 要抓指定颜色的物料，当然也可以不指定yanse
// 先让机械臂移动到一个俯瞰的位置，大概0, 90, 90, 0（发给下危机的角度）
// 直接发布到机械臂关节节点，给串口发送，然后
// 暂时根据物料的圆形属性，调用maincam的circle服务
// 等待正确的物料颜色，如果里画面中心太远，就把机械臂水平平移过去
// 到差不多正中心，然后根据距离下降高度。这个圆是物料的底面圆，所以不会太高，只会底，这个没什么关系
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stereo_msgs/msg/uint8.hpp>
#include <interfaces/srv/circle.hpp>

class Grab1Node : public rclcpp::Node
{
    Grab1Node() : Node("grab_node")
    {

        auto server_ = this->create_service<stereo_msgs::srv::UInt8>("grab1", std::bind(&Grab1Node::grab_callback, this, std::placeholders::_1, std::placeholders::_2));
        auto circle_client_ = this->create_client<interfaces::srv::Circle>("circle"); // 由maincam提供的服务，返回物料的圆心坐标和半径
        auto ik_client_ = this->create_client<interfaces::srv::Ik>("ik"); // 由maincam提供的服务，返回物料的圆心坐标和半径
        auto pub_joint = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        self.get_comm_joint();
    }

    self.get_comm_joint(){
        sensor_msgs::msg::JointState self.detect_joint ; // 待定读取逻辑
    }

    void grab_callback(const std::shared_ptr<stereo_msgs::srv::UInt8::Request> request, std::shared_ptr<stereo_msgs::srv::UInt8::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received grab request: %d", request->data);
        // 在这里添加抓取逻辑
  
        // 假设这里有一些机械臂关节状态的设置
        // 先看
        pub_joint->publish(self.detect_joint);
        // 再平移
        interfaces::srv::Circle::Request circle_request;
        circle_request.color = request->color; // 假设请求中包含颜色信息
        auto circle_future = circle_client_->async_send_request(circle_request);
        while (!circle_future.wait_for(std::chrono::seconds(1))) {
            // 等待响应
        }
        auto circle_response = circle_future.get();
        while(!circle_future.isCenter){
            // 根据结果处理一下
            // 如果圆心不在画面中心，平移机械臂
            pub_joint->publish(self.joint_cmd);
            // 再次观测
        }

        response->data = 1; // 假设抓取成功
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