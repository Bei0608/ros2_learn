#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>   //引入时间相关头文件

//使用命名空间，方便使用时间相关的类和函数
using namespace std::chrono_literals;

class TurtleCircle : public rclcpp::Node 
{

private:
    rclcpp::TimerBase::SharedPtr timer_; // 定时器指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 速度消息发布者指针
    
public:
    explicit TurtleCircle(const std::string & node_name) : Node(node_name) 
    {
        // 创建发布者，发布到"turtle1/cmd_vel"主题，队列长度为10
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // 创建定时器，每500毫秒调用一次timer_callback函数
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&TurtleCircle::timer_callback, this));
    }

private:
    // 定时器回调函数
    void timer_callback() 
    {
        // 创建一个Twist消息对象
        auto msg= geometry_msgs::msg::Twist();
        
        // 设置线速度和角速度，控制乌龟画圆
        msg.linear.x = 1.0;   // 线速度为1.0
        msg.angular.z = 0.5;  // 角速度为0.5

        // 发布速度消息
        publisher_->publish(msg);
    }

};

int main(int argc, char **argv) 
{
    // 初始化ROS 2客户端库
    rclcpp::init(argc, argv);

    // 创建TurtleCircle节点的共享指针
    auto node = std::make_shared<TurtleCircle>("turtle_circle");

    // 进入节点的事件循环
    rclcpp::spin(node);

    // 关闭ROS 2客户端库
    rclcpp::shutdown();
    return 0;
}
