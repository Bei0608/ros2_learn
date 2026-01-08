#include <QApplication>  // 导入画界面的工具
#include <QLabel>
#include <QString>
#include <sstream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp" // 导入消息格式

using SystemStatus = status_interfaces::msg::SystemStatus;

// 继承自 ROS 节点类，让你拥有收发消息的能力
class SysStatusDisplay : public rclcpp::Node {

private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QLabel* label_;

public:
    SysStatusDisplay() : Node("sys_status_display") 
    {
        // 步骤 1: 必须先创建 Label，防止回调函数访问空指针
        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>())); 
        label_->setWindowTitle("System Status"); // 给窗口起个名字
        label_->resize(300, 200); // 设置个大小，不然太小了
        label_->show();

        // 步骤 2: 创建订阅者
        subscription_ = this->create_subscription<SystemStatus>(
            "system_status",
            10,
            [&] 
            (const SystemStatus::SharedPtr msg)  -> void { 
            // 注意：这里仍然存在跨线程操作风险，但作为教程练习通常能跑通
            label_->setText(get_qstr_from_msg(msg));
            });

    };

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg) 
    {
        std::stringstream show_str;
        show_str
            << "======================系统状态可视化显示工具======================" << std::endl
            << "数据时间: \t" << msg->stamp.sec << "\ts\n"
            << "用户名: \t" << msg->host_name << "\t\n"
            << "CPU 使用率: \t" << msg->cpu_percent << "\t%\n"
            << "内存使用率: \t" << msg->memory_percent << "\t%\n"
            << "内存总大小: \t" << msg->memory_total << "\tMB\n"
            << "剩余有效内存: \t" << msg->memory_available << "\tMB\n"
            << "网络发送量: \t" << msg->net_sent << "\tMB\n"
            << "网络接收量: \t" << msg->net_recv << "\tMB\n"
            << "=============================================================" << std::endl;

        return QString::fromStdString(show_str.str());

    };

};


int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);  // 1. 初始化 ROS 2 环境
    QApplication app(argc, argv);  // 2. 初始化 Qt 界面环境
    // 3. 创建你的节点对象 (SysStatusDisplay)
    auto node = std::make_shared<SysStatusDisplay>();
    // 4. 【关键】开启一个新线程去跑 ROS
    // 为什么要新线程？因为下面的 app.exec() 是个死循环。
    // 如果不用新线程，ROS 的 spin 就没机会执行，收不到消息。
    std::thread spin_thread([&]() { rclcpp::spin(node); });
    spin_thread.detach(); // detach 表示“让这个线程自己在后台跑，不管它了”
    app.exec();  // 5. 启动 Qt 的界面循环 (程序会卡在这里，直到你关闭窗口)
    rclcpp::shutdown();   // 6. 退出清理
    return 0;
}