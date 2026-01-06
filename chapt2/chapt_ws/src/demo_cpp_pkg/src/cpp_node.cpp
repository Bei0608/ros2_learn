#include<string>
#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node
{
private:
    std::string name_;
    int age_;
public:
    PersonNode (const std::string & name_node,
        const std::string & name,
        const int age):Node(name_node)
    {
        this->name_ = name;
        this->age_ = age;
    };

    void eat(const std::string & food_name)
    {
        RCLCPP_INFO(this->get_logger(),"我是 %s 今年 %d 岁 我现在在吃 %s", 
            name_.c_str(),age_,food_name.c_str());
    };
};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PersonNode>("cpp_node","张三",20);
    node->eat("苹果");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}