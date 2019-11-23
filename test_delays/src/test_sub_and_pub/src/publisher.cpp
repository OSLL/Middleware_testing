#include <chrono>
#include <ctime>
#include <memory>
#include <fstream>
#include <iostream>
#include <vector>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
class Publisher : public rclcpp::Node
{
  public:
    Publisher(int length, int mcount)
    : Node("publisher"), time_list(mcount), count_(0), mcount(mcount)
    {
      mes = std::string(length, 'a');
      publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", createQoS());
      pid_t id = getpid();
      std::ofstream f_task("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", std::ios_base::out);
      if(!f_task.is_open()){
        RCLCPP_WARN_ONCE(this->get_logger(), "Erorr in adding to cpuset");
      }
      else{
        auto s = std::to_string(id);
        f_task.write(s.c_str(),s.length());
      }
      f_task.close();
      timer_ = this->create_wall_timer(
      0ms, std::bind(&Publisher::timer_callback, this));
    }

    ~Publisher(){
        std::ofstream file("publisher.txt");
        for(unsigned i=0; i<time_list.size();i++)
            file << time_list[i] << ' ';
        file << std::endl;
        file.close();
    }

  private:
    rclcpp::QoS createQoS(){
      rmw_qos_profile_t pr = rmw_qos_profile_default;
      pr.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      pr.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      pr.durability =  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      rclcpp::QoSInitialization QoSinit = rclcpp::QoSInitialization::from_rmw(pr);
      rclcpp::QoS test_QoS(QoSinit, pr);
      return test_QoS;
    }

    void timer_callback()
    {
      if(count_ == 0)
        usleep(4000000);
      auto message = std_msgs::msg::String();
      message.data = mes;
      high_resolution_clock::time_point t = high_resolution_clock::now();
      publisher_->publish(message);
      time_list[count_] = duration_cast<nanoseconds>(t.time_since_epoch()).count();
      ++count_;
      if(count_ == mcount) {
        timer_.reset();
      }
    }
    std::vector<unsigned long int> time_list;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string mes;
    size_t count_;
    size_t mcount;
  };

  int main(int argc, char * argv[])
  {
    if(argc < 3) {
      std::cout << "Missed length and/or message count" << std::endl;
      return 0;
    }
    int length = atoi(argv[1]);
    int mcount = atoi(argv[2]);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>(length, mcount));
    rclcpp::shutdown();
    return 0;
  }