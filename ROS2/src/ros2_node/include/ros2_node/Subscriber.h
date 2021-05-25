#pragma once
#include <sub_interface.hpp>
#include <string>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "ros2_node/msg/test_data.hpp"

class SubscriberNode: public rclcpp::Node{
public:
    SubscriberNode(std::string &topic, TestMiddlewareSub<ros2_node::msg::TestData::SharedPtr> *master);
    SubscriberNode();

protected:
    rclcpp::Subscription<ros2_node::msg::TestData>::SharedPtr _subscription;
    rclcpp::QoS createQoS();
};

class Subscriber: public TestMiddlewareSub<ros2_node::msg::TestData::SharedPtr> {
public:
    Subscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_prior);

    ~Subscriber();
    void callback(ros2_node::msg::TestData::SharedPtr msg);
    bool receive() override;
    short get_id(ros2_node::msg::TestData::SharedPtr &msg) override;
    unsigned long get_timestamp(ros2_node::msg::TestData::SharedPtr &msg) override;
private:
    rclcpp::executors::SingleThreadedExecutor *_exec = nullptr;
    std::shared_ptr<SubscriberNode> _ros_node;
    std::atomic_bool isReceived = false;
    std::atomic_int32_t _last_rec_id = -1;
};
