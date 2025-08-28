#include "rclcpp/rclcpp.hpp"
#include "autopatrol_interfaces/srv/speech_text.hpp"
#include <cstdlib>
#include <string>
#include <cstring>
#include <unistd.h>

class SpeakerNode : public rclcpp::Node {
public:
    SpeakerNode() : Node("speaker_node") {
        // 检查 espeak-ng 是否可用
        if (system("which espeak-ng > /dev/null 2>&1") != 0) {
            RCLCPP_ERROR(this->get_logger(), "espeak-ng 未安装，语音服务不可用");
            throw std::runtime_error("espeak-ng not installed");
        }
        
        service_ = this->create_service<autopatrol_interfaces::srv::SpeechText>(
            "speech_text",
            [this](const std::shared_ptr<autopatrol_interfaces::srv::SpeechText::Request> request,
                   std::shared_ptr<autopatrol_interfaces::srv::SpeechText::Response> response) {
                this->handleSpeechRequest(request, response);
            });
        
        RCLCPP_INFO(this->get_logger(), "语音服务已启动（使用 espeak-ng 命令行）");
    }
    
private:
    void handleSpeechRequest(
        const std::shared_ptr<autopatrol_interfaces::srv::SpeechText::Request> request,
        std::shared_ptr<autopatrol_interfaces::srv::SpeechText::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "播报: %s", request->text.c_str());
        
        // 创建安全的命令行参数
        std::string command = "espeak-ng -v zh \"";
        for (char c : request->text) {
            if (c == '"' || c == '`' || c == '$' || c == '\\' || c == ';') {
                // 转义特殊字符
                command += '\\';
            }
            command += c;
        }
        command += "\"";
        
        // 执行命令
        int result = std::system(command.c_str());
        
        if (WIFEXITED(result) && WEXITSTATUS(result) == 0) {
            RCLCPP_INFO(this->get_logger(), "语音播报成功");
            response->result = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "语音播报失败，返回码: %d", result);
            response->result = false;
        }
    }
    
    rclcpp::Service<autopatrol_interfaces::srv::SpeechText>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeakerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}