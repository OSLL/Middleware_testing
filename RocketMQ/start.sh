#!/bin/bash
cd $1
nohup sh bin/mqnamesrv &
sleep 5
export NAMESRV_ADDR=localhost:9876
nohup sh bin/mqbroker -n localhost:9876 &
sleep 10
clusters=$(./bin/mqadmin clusterList)
broker_addr=(${clusters[@]})
broker_addr=${broker_addr[15]}
./bin/mqadmin updateTopic -b $broker_addr -o true -t test_topic
./bin/mqadmin updateTopic -b $broker_addr -o true -t test_topic1
