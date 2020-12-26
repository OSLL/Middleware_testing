#pragma once
#include <string>
#include <atomic>
#include <ping_pong_interface.hpp>
#include "rclcpp/rclcpp.hpp"
#include "ros2_node/msg/test_data.hpp"
#include "Publisher.h"
#include "Subscriber.h"

class SubscriberPPNode : public SubscriberNode{
public:
    SubscriberPPNode(std::string &topic, TestMiddlewarePingPong<ros2_node::msg::TestData::SharedPtr> *master);
};

class PingPong : public TestMiddlewarePingPong<ros2_node::msg::TestData::SharedPtr>{
public:
    PingPong(std::string &topic1, std::string topic2, int msgCount, int prior,
             int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst);

    PingPong(std::string &topic1, std::string topic2, int msgCount, int prior,
             int cpu_index, std::string &filename, int topic_priority,
             int msInterval, int msgSizeMin, int msgSizeMax, int step,
             int before_step, bool isFirst);
    ~PingPong();

    bool receive() override;

    short get_id(ros2_node::msg::TestData::SharedPtr &msg) override;

    unsigned long get_timestamp(ros2_node::msg::TestData::SharedPtr &msg) override;

    void publish(short id, unsigned size) override;
    void callback(ros2_node::msg::TestData::SharedPtr msg);

private:
    void init();
    PublisherNode *_pub_node;
    rclcpp::executors::SingleThreadedExecutor *_exec = nullptr;
    std::shared_ptr<SubscriberPPNode> _sub_node;
    std::atomic_bool isReceived = false;
    std::atomic_int32_t _last_rec_id = -1;
};
