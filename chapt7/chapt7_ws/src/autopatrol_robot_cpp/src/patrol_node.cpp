#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include <chrono>
#include <vector>

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Transform = geometry_msgs::msg::Transform;

class PatrolNode : public rclcpp::Node{
public:
    PatrolNode():Node("patrol_node"){
        RCLCPP_INFO(this->get_logger(), "这是自动巡航节点！");

        this->declare_parameter<std::vector<double>>("initial_point", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("target_points", std::vector<double>{0.0, 0.0, 0.0});
        
        this->initial_point = this->get_parameter("initial_point").as_double_array();
        this->target_points = this->get_parameter("target_points").as_double_array();
    
    }
private:
    std::vector<double> initial_point;
    std::vector<double> target_points;

    void initRobotPose(){

    }
    std::vector<std::vector<double>> getTargetPoints(){

    }
    PoseStamped getPoseByXYYaw(double x, double y, double yaw){

    }
    void navigateToPose(const PoseStamped &target_pose){

    }
    Transform getCurrentPose(){
        
    }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
