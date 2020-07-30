#!/bin/bash
cd $1
nohup sh bin/mqnamesrv &
sleep 1
nohup sh bin/mqbroker -n localhost:9876 &
sleep 3
export NAMESRV_ADDR=localhost:9876
clusters=$(./bin/mqadmin clusterList)
broker_addr=(${clusters[@]})
broker_addr=${broker_addr[15]}
./bin/mqadmin updateTopic -b $broker_addr -t test_topic
./bin/mqadmin updateTopic -b $broker_addr -t test_topic1
