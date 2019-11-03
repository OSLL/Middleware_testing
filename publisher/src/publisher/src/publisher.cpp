#include <chrono>
#include <ctime>
#include <memory>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
class Publisher : public rclcpp::Node
{
  public:
    Publisher(int length=10)
    : Node("publisher"), out("publish.txt"), count_(0)
    {
      mes = std::string(length, 'a');
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      start_time = high_resolution_clock::now();
      nanoseconds ns = duration_cast<nanoseconds>(start_time.time_since_epoch());
      std::time_t t = duration_cast<seconds>(ns).count();
      out << std::ctime(&t);
      out << ns.count() % 1000000000 << std::endl;
      timer_ = this->create_wall_timer(
      0ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = mes;
      high_resolution_clock::time_point t = high_resolution_clock::now();
      publisher_->publish(message);
      out << duration_cast<nanoseconds>(t-start_time).count() << std::endl;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::ofstream out;
    high_resolution_clock::time_point start_time;
    std::string mes;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    int length;
    std::cin >> length;
    rclcpp::spin(std::make_shared<Publisher>(length));
    rclcpp::shutdown();
    return 0;
  }
