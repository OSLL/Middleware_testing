//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_TOPIC_H
#define CYCLONE_TEST_TOPIC_H

#include <dds/dds.h>
#include <Partisipant.h>



class Topic {
public:

    Topic() = default;

    Topic(Participant participant, std::string &topic_name, dds_topic_descriptor &topic_descriptor) {
        /* Create a Topic. */
        _topic_descriptor = &topic_descriptor;
        _topic = dds_create_topic (
                participant._participant, &topic_descriptor, topic_name.c_str(), nullptr, nullptr);

        _topic_name = topic_name;
        std::cout << "topic: " << dds_strretcode(-_topic) << std::endl;

        if (_topic < 0)
            DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-_topic));
    }

    dds_topic_descriptor _topic_descriptor;
    dds_entity_t _topic = 0;
    std::string _topic_name;

};



#endif //CYCLONE_TEST_TOPIC_H
