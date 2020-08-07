#!/bin/bash
cd $1
export NAMESRV_ADDR=localhost:9876
clusters=$(./bin/mqadmin clusterList)
broker_addr=(${clusters[@]})
cluster_name=${broker_addr[12]}
broker_addr=${broker_addr[15]}
./bin/mqadmin deleteTopic -c $cluster_name -t test_topic
./bin/mqadmin deleteTopic -c $cluster_name -t test_topic1
./bin/mqadmin deleteSubGroup -b $broker_addr -g subscribers
sh bin/mqshutdown broker
sh bin/mqshutdown namesrv
