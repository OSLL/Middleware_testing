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
    Subscriber(bool need_cpuset, int mcount): Node("subscriber"),time_list(mcount),id(getpid()), m_count(mcount),count(0)
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("test_topic", createQoS(), std::bind(&Subscriber::callback, this, std::placeholders::_1));
      if(need_cpuset){
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
    file <<"|||"<< std::endl;
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

    void callback(std_msgs::msg::String::SharedPtr msg)
    {
      auto rec_time = std::chrono::high_resolution_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(rec_time.time_since_epoch()).count();
      time_list[count] = ns;
      count++;
        //RCLCPP_INFO(this->get_logger(), "Package recieved: '%s', count = %d", msg->data.c_str(), count);
      if(count == m_count) {
          RCLCPP_INFO(this->get_logger(), "Last package recieved by %s", get_name());
          rclcpp::shutdown();
      }
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
  bool need_cpuset;
  std::istringstream(argv[1]) >> std::boolalpha >> need_cpuset;
  int mcount = atoi(argv[2]);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(need_cpuset,mcount));
  rclcpp::shutdown();
  return 0;
}