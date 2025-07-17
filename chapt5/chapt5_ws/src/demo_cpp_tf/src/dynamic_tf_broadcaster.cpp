#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>

using namespace std::chrono_literals;

class DynamicTFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    DynamicTFBroadcaster() : Node("dynamic_tf_broadcaster_node")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(10ms, std::bind(&DynamicTFBroadcaster::publish_tf, this));
    }
    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped transfrom;
        transfrom.header.stamp = this->get_clock()->now();
        transfrom.header.frame_id = "map";
        transfrom.child_frame_id = "base_link";
        transfrom.transform.translation.x = 2.0;
        transfrom.transform.translation.y = 3.0;
        transfrom.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 30 * M_PI / 180);
        transfrom.transform.rotation = tf2::toMsg(quat);
        broadcaster_->sendTransform(transfrom);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}