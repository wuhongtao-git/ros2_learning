#include "rclcpp/rclcpp.hpp"
#include <string.h>
using namespace std;

class PersonNode : public rclcpp::Node{
private:
    int age_;
    string name_;
public:
    PersonNode(const string &node_name, const string &name, const int &age):Node(node_name){
        this->age_ = age;
        this->name_ = name;
    }
    void eat(const string &food_name){
        RCLCPP_INFO(this->get_logger(), "woshi %s, jinnian %d sui, xihuanchi %s", name_.c_str(), age_, food_name.c_str());
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<PersonNode>("cpp_node", "wht", 24);
    node->eat("xia");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}