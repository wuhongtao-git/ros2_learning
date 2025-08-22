#include "rclcpp/rclcpp.hpp"

class Speaker : public rclcpp::Node{
public:
    Speaker() : Node("speaker_node"){
        RCLCPP_INFO(this->get_logger(), "语言节点启动");
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Speaker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}