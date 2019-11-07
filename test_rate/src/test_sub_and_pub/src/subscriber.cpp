#include <chrono>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class Subscriber : public rclcpp::Node
{
  public:
    Subscriber(char *add_to_cpuset): Node("test_subscriber"),id(getpid())
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("test_topic", createQoS(), std::bind(&Subscriber::callback, this, std::placeholders::_1));
      if(std::string(add_to_cpuset) == "true"){
        std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
        if(!f_task.is_open()){
          RCLCPP_WARN_ONCE(this->get_logger(), "Erorr in adding to cpuset");
        }
        else{
          auto s = std::to_string(id);
          f_task.write(s.c_str(),s.length());
        }
        f_task.close();
      }
    }

  ~Subscriber(){

    std::ofstream file(std::string(get_name())+".txt");
    for(unsigned i=0; i<time_list.size();i++)
      file << time_list[i] << ' ';
    file << std::endl;
    file.close();
  }

  private:

    rclcpp::QoS createQoS(){
      rmw_qos_profile_t pr = rmw_qos_profile_default;
      pr.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      pr.depth = SIZE_MAX+1;
      pr.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      pr.durability =  RMW_QOS_POLICY_DURABILITY_VOLATILE;
      pr.deadline.nsec = UINT64_MAX+1;
      pr.lifespan = pr.deadline;
      rclcpp::QoSInitialization QoSinit = rclcpp::QoSInitialization::from_rmw(pr);
      rclcpp::QoS test_QoS(QoSinit, pr);
      return test_QoS;
    }

    void callback(std_msgs::msg::String::SharedPtr msg)
    {
      auto rec_time = std::chrono::high_resolution_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(rec_time.time_since_epoch()).count();
      time_list.push_back(ns);
      RCLCPP_INFO(this->get_logger(), "I heard: '%s - %lu'", msg->data.c_str(), time_list.size());
    }

    std::vector<unsigned long int> time_list;
    cpu_set_t my_set;
    pid_t id;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  if(argc < 2){
    std::cout << "Missed argument" << std::endl;
    return 0;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(argv[1]));
  rclcpp::shutdown();
  return 0;
}