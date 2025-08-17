#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

class Nav2InitialPose : public nav2_util::LifecycleNode
{
public:
  Nav2InitialPose() : LifecycleNode("nav2_initial_pose_node")
  {
    // 创建生命周期发布者
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating...");
    
    // 激活发布者
    initial_pose_pub_->on_activate();
    
    // 设置初始位姿
    set_initial_pose();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    initial_pose_pub_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  void set_initial_pose()
  {
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    
    // 填充位姿消息
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->now();
    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.position.y = 0.0;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;
    
    // 设置协方差（可选）
    pose_msg.pose.covariance[0] = 0.25;  // x的方差
    pose_msg.pose.covariance[7] = 0.25;  // y的方差
    pose_msg.pose.covariance[35] = 0.06853892326654787;  // 偏航角的方差
    
    // 发布初始位姿
    initial_pose_pub_->publish(pose_msg);
    RCLCPP_INFO(get_logger(), "Initial pose published");
  }

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Nav2InitialPose>();
  
  // 手动管理生命周期状态转换
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  // 保持节点运行
  rclcpp::spin(node->get_node_base_interface());
  
  // 在关闭前停用节点
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  
  rclcpp::shutdown();
  return 0;
}