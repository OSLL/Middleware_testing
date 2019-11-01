#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber(): Node("test_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "I STARTED");
      subscription_ = this->create_subscription<std_msgs::msg::String>("test_topic", 10, std::bind(&Subscriber::callback, this, std::placeholders::_1));
    }

  private:
    void callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      auto rec_time = std::chrono::system_clock::now();
      auto rec_time__ms = std::chrono::time_point_cast<std::chrono::milliseconds>(rec_time);
      long ms = std::chrono::duration_cast<std::chrono::milliseconds>(rec_time__ms.time_since_epoch()).count();
      std::ofstream file("RecTime.txt", std::ios::app);
      file << ms << std::endl;
      file.close();
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'\n Time: %ld", msg->data.c_str(), ms);

    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}