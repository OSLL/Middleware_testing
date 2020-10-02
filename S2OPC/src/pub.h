#ifndef PUB_H
#define PUB_H

#include <string>

extern "C" void configure_server(std::string& topic, long int delay_ms);

extern "C" void stop_server();

#endif
