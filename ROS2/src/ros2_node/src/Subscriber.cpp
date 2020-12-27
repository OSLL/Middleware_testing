#include "../include/ros2_node/Subscriber.h"

SubscriberNode::SubscriberNode(std::string &topic, TestMiddlewareSub<ros2_node::msg::TestData::SharedPtr> *master):
        Node("subscriber") {
    auto qos = createQoS();
    _subscription = this->create_subscription<ros2_node::msg::TestData>(
            topic, qos, std::bind(&Subscriber::callback, (Subscriber *)master,
                              std::placeholders::_1));
}

SubscriberNode::SubscriberNode():
        Node("subscriber"){
}

rclcpp::QoS SubscriberNode::createQoS() {
    rmw_qos_profile_t pr = rmw_qos_profile_default;
    pr.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    pr.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    pr.durability =  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rclcpp::QoSInitialization QoSinit = rclcpp::QoSInitialization::from_rmw(pr);
    rclcpp::QoS test_QoS(QoSinit, pr);
    return test_QoS;
}


void Subscriber::callback(ros2_node::msg::TestData::SharedPtr msg) {
    unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    //std::cout << msg->id << std::endl;
    _last_rec_id = get_id(msg);
    write_received_msg(msg, proc_time);
    isReceived = true;
}

Subscriber::Subscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename,
                       int topic_prior):
                       TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_prior) {

    rclcpp::init(0, nullptr);
    _exec = new rclcpp::executors::SingleThreadedExecutor();
    _ros_node = std::make_shared<SubscriberNode>(_topic_name, this);

}

Subscriber::~Subscriber() {
    delete _exec;
    rclcpp::shutdown();
}

bool Subscriber::receive() {
    isReceived = false;
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    _exec->spin_node_once(_ros_node, std::chrono::milliseconds(1));
    if (isReceived) {
        _read_msg_time[_last_rec_id] -= cur_time;
    }
    return isReceived;
}

short Subscriber::get_id(ros2_node::msg::TestData::SharedPtr &msg) {
    return msg->id;
}

unsigned long Subscriber::get_timestamp(ros2_node::msg::TestData::SharedPtr &msg) {
    return msg->timestamp;
}

