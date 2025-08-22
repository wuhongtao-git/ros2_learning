#include "rclcpp/rclcpp.hpp"
#include "autopatrol_interfaces/srv/speech_text.hpp"

using SpeechText = autopatrol_interfaces::srv::SpeechText;

using namespace std::placeholders;

class Speaker : public rclcpp::Node{
public:
    Speaker() : Node("speaker_node"){
        RCLCPP_INFO(this->get_logger(), "语言节点启动");
        speaker_service_ = this->create_service<SpeechText>(
            "speech_text",
            std::bind(&Speaker::handleSpeechRequest, this, _1, _2)
        );
    }
    ~Speaker(){

    }
private:
    void handleSpeechRequest(
    const std::shared_ptr<SpeechText::Request> request,
          std::shared_ptr<SpeechText::Response> response){
        RCLCPP_INFO(this->get_logger(), "播报：%s", request->text.c_str());

        response->result = true;
    }

    rclcpp::Service<SpeechText>::SharedPtr speaker_service_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Speaker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}