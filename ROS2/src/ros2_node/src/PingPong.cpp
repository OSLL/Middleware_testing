#include "../include/ros2_node/PingPong.h"

PingPong::PingPong(std::string &topic1, std::string topic2, int msgCount, int prior, int cpu_index,
                   std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst):
        TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
                               filename, topic_priority, msInterval, msgSize, isFirst)
                   {
    init();
}

PingPong::PingPong(std::string &topic1, std::string topic2, int msgCount, int prior, int cpu_index,
                   std::string &filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax, int step,
                   int before_step, bool isFirst):
                   TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                          msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
{
    init();
}

PingPong::~PingPong() {
    delete _pub_node;
    delete _exec;
    rclcpp::shutdown();
}

bool PingPong::receive() {
    isReceived = false;
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    _exec->spin_node_once(_sub_node, std::chrono::milliseconds(1));
    if (isReceived) {
        _read_msg_time[_last_rec_id] -= cur_time;
    }
    return isReceived;
}
void PingPong::publish(short id, unsigned int size) {
    _write_msg_time[id] = _pub_node->publish(id, size);
}

void PingPong::init() {
    if(!_isFirst)
        std::swap(_topic_name1, _topic_name2);
    rclcpp::init(0, nullptr);
    _exec = new rclcpp::executors::SingleThreadedExecutor();
    _sub_node = std::make_shared<SubscriberPPNode>(_topic_name2, this);
    _pub_node = new PublisherNode(_topic_name1);
}

void PingPong::callback(ros2_node::msg::TestData::SharedPtr msg) {
    unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    //std::cout << msg->id << std::endl;
    _last_rec_id = get_id(msg);
    write_received_msg(msg);
    _read_msg_time[_last_rec_id] = proc_time;
    isReceived = true;
}

short PingPong::get_id(ros2_node::msg::TestData::SharedPtr &msg) {
    return msg->id;
}

unsigned long PingPong::get_timestamp(ros2_node::msg::TestData::SharedPtr &msg) {
    return msg->timestamp;
}

SubscriberPPNode::SubscriberPPNode(std::string &topic,
                                   TestMiddlewarePingPong<ros2_node::msg::TestData::SharedPtr> *master)
        : SubscriberNode() {
    auto qos = createQoS();
    _subscription = this->create_subscription<ros2_node::msg::TestData>(
            topic, qos, std::bind(&PingPong::callback, (PingPong *)master,
                                  std::placeholders::_1));
}
