#include <chrono>
#include <ctime>
#include <memory>
#include <fstream>
#include <iostream>

#include <unistd.h>

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
      rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
      qos_profile.depth = 5000;
      publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", qos_profile);
      pid_t id = getpid();
      std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
      if(!f_task.is_open()){
        RCLCPP_INFO(this->get_logger(), "FILE OPEN ERORR");
      }
      else{
        auto s = std::to_string(id);
        f_task.write(s.c_str(),s.length());
      }
      f_task.close();
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
      out << duration_cast<nanoseconds>(t.time_since_epoch()).count() << std::endl;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::ofstream out;
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
