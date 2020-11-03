package main

import (
	"flag"
	"log"
	"os"
	"io/ioutil"
	"encoding/json"


	"pulsar_test/TestPublisher"
	"pulsar_test/TestSubscriber"
	"pulsar_test/TestPingPong"
)

type Args struct{
	Topic []string `json:"topic"`
	Res_filenames []string `json:"res_filenames"`
	M_count int `json:"m_count"`
	Min_msg_size int `json:"min_msg_size"`
	Max_msg_size int `json:"max_msg_size"`
	Step int `json:"step"`
	Msgs_before_step int `json:"msgs_before_step"`
	Priority []int `json:"priority"`
	Cpu_index []int `json:"cpu_index"`
	Topic_priority int `json:"topic_priority"`
	Interval int `json:"interval"`
}

func main() {
	var config_file = flag.String("c", "", "is required argument with config path")
	var node_type = flag.String("t", "", "is required argument with type of node")
	var isFirst = flag.Bool("first", false, "is required argument if ping_pong type is specified with type of node")


	log.SetFlags(0)
	flag.Parse()

	var config Args

	jsonFile, err := os.Open(*config_file)
	if err != nil {
		log.Fatal(err)
	}
	defer jsonFile.Close()

	byteValue, _ := ioutil.ReadAll(jsonFile)
	err = json.Unmarshal([]byte(byteValue), &config)
	if err != nil {
		log.Fatal(err)
	}
	if *node_type == "publisher"{
		var pub = TestPublisher.New(config.Topic[0], config.M_count, config.Priority[0], config.Cpu_index[0], config.Min_msg_size, config.Max_msg_size, config.Step, config.Interval, config.Msgs_before_step, config.Res_filenames[0], config.Topic_priority)

		defer pub.Close()

		pub.StartTest()
	} else if *node_type == "subscriber" {
		var sub = TestSubscriber.New(config.Topic[0], config.M_count, config.Priority[1], config.Cpu_index[1], config.Max_msg_size, config.Step, config.Interval, config.Msgs_before_step, config.Res_filenames[1], config.Topic_priority)

		defer sub.Close()

		sub.StartTest()
	} else if *node_type == "ping_pong" {
		var filename, topic1, topic2 string
		var prior, cpu_index int
		if *isFirst {
			filename = config.Res_filenames[0]
			topic1 = config.Topic[0]
			topic2 = config.Topic[1]
			prior = config.Priority[0]
			cpu_index = config.Cpu_index[0]
		} else {
			filename = config.Res_filenames[1]
			topic1 = config.Topic[1]
			topic2 = config.Topic[0]
			prior = config.Priority[1]
			cpu_index = config.Cpu_index[1]
		}
		var pingpong = TestPingPong.New(topic1, topic2, config.M_count, prior, cpu_index, config.Min_msg_size, config.Interval, filename, config.Topic_priority, *isFirst)

		defer pingpong.Close()

		pingpong.StartTest()
	}
}
