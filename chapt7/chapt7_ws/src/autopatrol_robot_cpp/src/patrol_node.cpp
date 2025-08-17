#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Transform = geometry_msgs::msg::Transform;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;


class PatrolNode : public rclcpp::Node{
public:
    PatrolNode():Node("patrol_node"){
        RCLCPP_INFO(this->get_logger(), "这是自动巡航节点！");

        this->declare_parameter<std::vector<double>>("initial_point", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("target_points", std::vector<double>{0.0, 0.0, 0.0, 1.0, 1.0, 1.57});
        
        this->initial_point_ = this->get_parameter("initial_point").as_double_array();
        this->target_points_ = this->get_parameter("target_points").as_double_array();
    
        this->initial_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
        initRobotPose();
    }
private:

    void initRobotPose(){
        auto pose = getPoseByXYYaw(initial_point_[0], initial_point_[1], initial_point_[2]);
        

        PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.pose.pose = pose.pose;

        msg.pose.covariance[0] = 0.25;
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = 0.06853892326654787;
        
        rclcpp::sleep_for(500ms);
        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Initial pose set");
    }
    std::vector<std::vector<double>> getTargetPoints(){
        std::vector<std::vector<double>> points;
        for(size_t i = 0; i < target_points_.size(); i += 3){
            double x = target_points_[i];
            double y = target_points_[i + 1];
            double z = target_points_[i + 2];
            points.push_back({x, y, z});
            RCLCPP_INFO(this->get_logger(), "目标点： %d -> (%.2f, %.2f, %.2f)", i/3, x, y, z);
        }
        return points;
    }
    PoseStamped getPoseByXYYaw(double x, double y, double yaw){
        PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }
    void navigateToPose(const PoseStamped &target_pose){
        if(!nav_action_client_->wait_for_action_server(5s)){
            RCLCPP_ERROR(this->get_logger(), "未连接到导航服务器");
            return; 
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleNavigate::SharedPtr &goal_handle){
            if(!goal_handle){
                RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "目标已接受");
            }
        };
        send_goal_options.feedback_callback = [this](GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback){
            RCLCPP_INFO(this->get_logger(), "预计%.1f秒后到达", feedback->estimated_time_remaining.sec);
        };
        send_goal_options.result_callback = [this](const GoalHandleNavigate::WrappedResult &result){
            switch(result.code){
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "成功到达目标点");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "导航失败");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "导航被取消");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "未知结果");
                    break;
            }
        };

        auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);
        
        auto goal_handle = future_goal_handle.get();
        if(!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "导航目标被拒绝");
            return;
        }
        auto result_future = nav_action_client_->async_get_result(goal_handle);
        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
        
        if(result != rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "导航过程失败");
            return;
        }
    
        auto wrapped_result = result_future.get();

        switch(wrapped_result.code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "导航成功完成");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "导航失败");
                break;
        }
    
    }
    Transform getCurrentPose(){
        try{
            auto transfrom = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero
            );
            return transfrom.transform;
        }catch (tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "获取位姿失败：%s", ex.what());
            return Transform();
        }
    }
    std::vector<double> initial_point_;
    std::vector<double> target_points_;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
