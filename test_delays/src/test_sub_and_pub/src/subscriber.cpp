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
    Subscriber(char *add_to_cpuset, int mcount): Node("subscriber"),id(getpid()), m_count(mcount),count(0)
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
    std::ofstream file("subscriber.txt", std::ofstream::app);
    for(unsigned i=0; i<time_list.size();i++)
      file << time_list[i] << ' ';
    file << std::endl << "-----------"<<std::endl;
    file.close();
  }

  private:
    rclcpp::QoS createQoS(){
      rmw_qos_profile_t pr = rmw_qos_profile_default;
      pr.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      pr.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      pr.durability =  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      /*pr.deadline.nsec = UINT64_MAX+1;
      pr.lifespan = pr.deadline;*/
      rclcpp::QoSInitialization QoSinit = rclcpp::QoSInitialization::from_rmw(pr);
      rclcpp::QoS test_QoS(QoSinit, pr);
      return test_QoS;
    }

    void callback(std_msgs::msg::String::SharedPtr msg)
    {
      count++;
      auto rec_time = std::chrono::high_resolution_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(rec_time.time_since_epoch()).count();
      time_list.push_back(ns);
      RCLCPP_INFO(this->get_logger(), "I heard: '%s - %lu'", msg->data.c_str(), time_list.size());
      if(count == m_count)
          rclcpp::shutdown();
    }

    std::vector<unsigned long int> time_list;
    cpu_set_t my_set;
    pid_t id;
    int m_count;
    int count;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  if(argc < 3){
    std::cout << "Missed argument" << std::endl;
    return 0;
  }
  int mcount = atoi(argv[2]);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(argv[1],mcount));
  rclcpp::shutdown();
  return 0;
}