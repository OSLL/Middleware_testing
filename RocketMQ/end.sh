#!/bin/bash
cd $1
export NAMESRV_ADDR=localhost:9876
clusters=$(./bin/mqadmin clusterList)
broker_addr=(${clusters[@]})
broker_addr=${broker_addr[15]}
./bin/mqadmin deleteTopic -b $broker_addr -t test_topic
./bin/mqadmin deleteTopic -b $broker_addr -t test_topic1
sh bin/mqshutdown broker
sh bin/mqshutdown namesrv
