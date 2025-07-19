#include <memory>
#include <string>
#include <functional>
#include <future>
#include <sstream>
#include <iostream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciActionClient : public rclcpp::Node
{
private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal()
    {
        this->timer_->cancel();
        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action erver is not available afer waiting");
            rclcpp::shutdown();
        }
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;
        RCLCPP_INFO(this->get_logger(), "Send goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal is accepted by server, waiting for result");
        }
    }
    void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Fibonacci>>, std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto num : feedback->partial_sequence)
        {
            ss << num << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknow result code");
            break;
        }
        std::stringstream ss;
        ss << "Result reveived: ";
        for (auto num : result.result->sequence)
        {
            ss << num << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }

public:
    FibonacciActionClient() : Node("fibonacci_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci");
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FibonacciActionClient::send_goal, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout << "Client exit" << std::endl;
    return 0;
}