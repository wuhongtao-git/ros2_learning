#include <memory>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciActionServer : public rclcpp::Node
{
private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    }
    rclcpp_action::CancelResponse handle_cancle(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accept(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "accept the goal");
        auto td = std::thread(std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1), goal_handle);
        td.detach();
    }
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        goal_handle->execute();
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto &sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<Fibonacci::Result>();

        for (int i = 1; i < goal->order && rclcpp::ok(); i++)
        {
            if (goal_handle->is_canceling())
            {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            sequence.push_back(sequence[i] + sequence[i - 1]);
    
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "i = %d, Publish feedback", i);

            loop_rate.sleep();
        }
        if (rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

public:
    FibonacciActionServer() : Node("fibonacci_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化fibonacci服务器");
        this->action_server_ = rclcpp_action::create_server<Fibonacci>(this, "fibonacci",
                                                                       std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                                       std::bind(&FibonacciActionServer::handle_cancle, this, std::placeholders::_1),
                                                                       std::bind(&FibonacciActionServer::handle_accept, this, std::placeholders::_1));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}