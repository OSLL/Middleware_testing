#include "../include/ros2_node/Publisher.h"

PublisherNode::PublisherNode(std::string &topic)
: Node("publisher"){
    auto qos = createQoS();
    _publisher = this->create_publisher<ros2_node::msg::TestData>(topic, qos);
}

unsigned long PublisherNode::publish(short id, unsigned int size) {
    auto msg = ros2_node::msg::TestData();
    msg.id = id;
    msg.data = std::string(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    msg.timestamp = cur_time;
    _publisher->publish(msg);
    return std::chrono::duration_cast<std::chrono::nanoseconds>
                   (std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
}

rclcpp::QoS PublisherNode::createQoS() {

    rmw_qos_profile_t pr = rmw_qos_profile_default;
    pr.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    pr.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    pr.durability =  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rclcpp::QoSInitialization QoSinit = rclcpp::QoSInitialization::from_rmw(pr);
    rclcpp::QoS test_QoS(QoSinit, pr);
    return test_QoS;

}

Publisher::Publisher(std::string topic, int msgCount, int prior, int cpu_index, int min_msg_size,
                     int max_msg_size, int step, int interval, int msgs_before_step, std::string &filename,
                     int topic_priority): TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size,
                                                            step, interval, msgs_before_step, filename, topic_priority){
    rclcpp::init(0, nullptr);
    _ros_node = new PublisherNode(topic);

}

unsigned long Publisher::publish(short id, unsigned int size) {
    // std::cout << "sent "<<id<<std::endl;
    return _ros_node->publish(id, size);
}

Publisher::~Publisher() {

    rclcpp::shutdown();
}
