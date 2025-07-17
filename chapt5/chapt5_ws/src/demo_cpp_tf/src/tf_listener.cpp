#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> listener;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    TFListener() : Node("tf_listener")
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        timer_ = create_wall_timer(5s, std::bind(&TFListener::getTransForm, this));
    }
    void getTransForm()
    {
        try{
            const auto transfrom = buffer_->lookupTransform(
                "base_link", "target_point", this->get_clock()->now(),
                rclcpp::Duration::from_seconds(1.0f));
            const auto &translation = transfrom.transform.translation;
            const auto &rotation = transfrom.transform.rotation;
            double yew, pitch, roll;
            tf2::getEulerYPR(rotation, yew, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "平移分量: (%f, %f, %f)", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "旋转分量: (%f, %f, %f)", roll, pitch, yew);
        }catch(tf2::TransformException &ex){
            RCLCPP_INFO(this->get_logger(), "异常: %s", ex.what());
        }
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}