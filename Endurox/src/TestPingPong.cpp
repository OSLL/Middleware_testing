#include "TestPingPong.h"

TestPingPong* pingpong;

bool my_msg = true;

static void handler(char* data, long len, long flags){
    my_msg = !my_msg;
    if(!my_msg)
	return;
    pingpong->msgHandler(data, len, flags);
}

TestPingPong::TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
	    TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
		        filename, topic_priority, msInterval, msgSize, isFirst)
{
    if(!isFirst){
        std::swap(_topic_name1, _topic_name2);
	my_msg = false;
    }

    if ((strbuf = tpalloc("STRING", NULL, msgSize+200)) == NULL) {
         fprintf(stderr, "Failed to alloc the buffer - %s\n",
                 tpstrerror(tperrno));
    }

    for(int i=0; i<_msgs.size(); ++i)
	_msgs[i] = new char[msgSize+200];
    pingpong = this;
    auto err = tpsetunsol(handler);
    if(err == TPUNSOLERR){
	fprintf(stderr, "Failed to tpsetunsol - %s\n",
                        tpstrerror(tperrno));
    }

}

TestPingPong::~TestPingPong()
{
    tpterm();
}

bool TestPingPong::receive() {
    tpchkunsol();
    int rec = n_received - n_before;
    n_before = n_received;
    return rec;
}

short TestPingPong::get_id(char* &msg) {
    try{
    nlohmann::json js = nlohmann::json::parse(msg);
    return js["id"];
    }
    catch(...){
    return 0;
    }
}

unsigned long TestPingPong::get_timestamp(char* &msg) {
    try{
    nlohmann::json js = nlohmann::json::parse(msg);
    return js["timestamp"];
    }
    catch(...){
    return 0;
    }
}

void TestPingPong::publish(short id, unsigned size) {
    std::string data(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    nlohmann::json jmsg;
    jmsg["id"] = id;
    jmsg["timestamp"] = cur_time;
    jmsg["msg"] = data;
    std::string jstr = jmsg.dump();
    strcpy(strbuf, jstr.c_str());
    if(tpbroadcast(NULL, NULL, NULL, strbuf, 0, TPSIGRSTRT) == -1)
	fprintf(stderr, "Failed to tpbroadcast - %s\n",
                        tpstrerror(tperrno));
}

void TestPingPong::msgHandler(char* data, long len, long flags){
    strcpy(_msgs[n_received], data);
    _recieve_timestamps[n_received] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    ++n_received;
}
