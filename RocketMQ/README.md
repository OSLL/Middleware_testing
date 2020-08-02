# RocketMQ testing guid

## Getting RocketMQ

1. Firstly you need to install java and maven(ver. > 3.0), for this project you need version jdk-11 and maven should use jdk-11
2. Get server and broker:
    * ```wget https://apache-mirror.rbc.ru/pub/apache/rocketmq/4.7.1/rocketmq-all-4.7.1-bin-release.zip```
    * ```unzip rocketmq-all-4.7.1-bin-release.zip```
    
## Building project

1. ```cd dds_testing/RocketMQ/```
2. ```mvn package```

## Running program
Running nsmeserver and broker: ```./start.sh <PATH_TO_ROCKETMQ>```
To run the program use command: ```java -jar ./target/RocketMQ-1.0-jar-with-dependencies.jar```
To get info about parameters use: ```java -jar ./target/RocketMQ-1.0-jar-with-dependencies.jar --help```
Shutting down nsmeserver and broker: ```./end.sh <PATH_TO_ROCKETMQ>```


