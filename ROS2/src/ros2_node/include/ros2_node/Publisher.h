#pragma once
#include <pub_interface.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ros2_node/msg/test_data.hpp"

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(std::string &topic);

    unsigned long publish(short id, unsigned size);


private:
    rclcpp::QoS createQoS();

    rclcpp::Publisher<ros2_node::msg::TestData>::SharedPtr _publisher;
};

class Publisher: public TestMiddlewarePub{
public:
    Publisher(std::string topic,  int msgCount, int prior, int cpu_index,
              int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
              std::string &filename, int topic_priority);

    ~Publisher();

    unsigned long publish(short id, unsigned size);

private:
    PublisherNode *_ros_node;
};
