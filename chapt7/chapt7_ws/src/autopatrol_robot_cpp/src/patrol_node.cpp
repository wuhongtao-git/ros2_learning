#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>

class PatrolNode : public rclcpp::Node{
public:
    PatrolNode():Node("patrol_node"){
        RCLCPP_INFO(this->get_logger(), "这是自动巡航节点！");
    }
private:

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
