#include <chrono>
#include <fstream>
#include <string>

#include <unistd.h>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class Subscriber : public rclcpp::Node
{
  public:
    Subscriber(): Node("test_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "I STARTED, %u", getpid());
      subscription_ = this->create_subscription<std_msgs::msg::String>("test_topic", 1000, std::bind(&Subscriber::callback, this, std::placeholders::_1));
      id = getpid();
      std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
      if(!f_task.is_open()){
        RCLCPP_INFO(this->get_logger(), "FILE OPEN ERORR");
      }
      else{
        auto s = std::to_string(id);
        f_task.write(s.c_str(),s.length());
      }
      f_task.close();
    }

  private:
    void callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      auto rec_time = std::chrono::high_resolution_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(rec_time.time_since_epoch());
      std::ofstream file("RecTime.txt", std::ios::app);
      file << ns.count() << std::endl;
      file.close();
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'\n Time: %lld\n\n", msg->data.c_str(), ns);
    }

    cpu_set_t my_set;
    pid_t id;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}