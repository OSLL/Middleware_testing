package TestPingPong

import(
	"strconv"
	"strings"
	"log"
	"os"
	"syscall"
	"time"
	"encoding/json"

	"github.com/nsqio/go-nsq"
)
const(
	timeout = 20*time.Second
)

type info struct{
	Msg msg_info `json:"msg"`
}

type msg_info struct{
	Id int `json:"id"`
	Receive_timestamp int64 `json:"recieve_timestamp"`
	Sent_time int64 `json:"sent_time"`
	Delay int64 `json:"delay"`
}

type msg struct{
	Id int `json:"id"`
	Sent_time int64 `json:"sent_time"`
	Msg string `json:"msg"`
}

type TestPingPong struct {
	topic1 string
	topic2 string
	msgCount int
	prior int
	cpu_index int
	msgSize int
	interval int
	filename string
	topic_priority int
	isFirst bool
	writer *nsq.Producer
	reader *nsq.Consumer
	msgs [][]byte
	receive_timestamp []int64
	n_received *int
	rec_before *int
}

func New(topic1 string, topic2 string, msgCount int, prior int, cpu_index int, msgSize int, interval int, filename string, topic_priority int, isFirst bool) TestPingPong{
	pid := os.Getpid()
	if prior >= 0 {
		err := syscall.Setpriority(syscall.PRIO_PROCESS, pid, prior)
		if err != nil {
			log.Fatal(err)
		}
	}
	if cpu_index >= 0 {
		err := os.MkdirAll("/sys/fs/cgroup/cpuset/sub_cpuset", os.ModePerm)
		if err != nil && err != os.ErrExist {
			log.Fatal(err)
		}
		f_cpu, err := os.OpenFile("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpus", os.O_WRONLY, 0577)
		if err != nil {
			log.Fatal(err)
		}
		defer f_cpu.Close()
		f_cpu.Write([]byte(strconv.Itoa(cpu_index)))

		f_exclusive, err := os.OpenFile("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpu_exclusive", os.O_WRONLY, 0577)
		if err != nil {
			log.Fatal(err)
		}
		defer f_exclusive.Close()
		f_exclusive.Write([]byte("1"))

		f_mem, err := os.OpenFile("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.mems", os.O_WRONLY, 0577)
		if err != nil {
			log.Fatal(err)
		}
		defer f_mem.Close()
		f_mem.Write([]byte("0"))

		f_task, err := os.OpenFile("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", os.O_WRONLY, 0577)
		if err != nil {
			log.Fatal(err)
		}
		defer f_task.Close()
		f_task.Write([]byte(strconv.Itoa(pid)))
	}
	config := nsq.NewConfig()
	writer, err := nsq.NewProducer("127.0.0.1:4150", config)
        if err != nil {
                log.Fatal(err)
        }
	reader, err := nsq.NewConsumer(topic2, "ch", config)
        if err != nil {
                log.Fatal(err)
        }
	pingpong := TestPingPong{topic1, topic2, msgCount, prior, cpu_index, msgSize, interval, filename, topic_priority, isFirst, writer, reader, make([][]byte, msgCount, msgCount), make([]int64, msgCount, msgCount), new(int), new(int)}
	reader.AddHandler(nsq.HandlerFunc(func(msg *nsq.Message) error {
		if *pingpong.n_received >= pingpong.msgCount {
			return nil
		}
		pingpong.msgs[*pingpong.n_received] = msg.Body
		pingpong.receive_timestamp[*pingpong.n_received] = time.Now().UnixNano()
		*pingpong.n_received += 1
		return nil
	}))
	err = reader.ConnectToNSQD("127.0.0.1:4150")
	if err != nil {
                log.Fatal(err)
        }
	return pingpong
}

func (pingpong TestPingPong) StartTest() int {
	var start_timeout, end_timeout int64
	isTimeoutEx := false
	time.Sleep(4*time.Second)
	for i := 0; i < pingpong.msgCount; i++ {
		if pingpong.isFirst {
			pingpong.publish(i, pingpong.msgSize)
		}
		start_timeout = time.Now().UnixNano()
		end_timeout = start_timeout
		not_received := true
		for not_received {
			if pingpong.receive() {
				not_received = false
			} else {
				end_timeout = time.Now().UnixNano()
				if (end_timeout - start_timeout > int64(timeout)) {
					isTimeoutEx = true
					break
				}
			}
			time.Sleep(time.Millisecond)
		}
		if isTimeoutEx {
			break
		}
		if !pingpong.isFirst {
			pingpong.publish(i, pingpong.msgSize)
		}
	}
	pingpong.toJson();
	return 0;
}

func (pingpong TestPingPong) toJson(){
	n := len(pingpong.msgs)
	info := make([]info, n, n)
	for i := 0; i<n; i++{
		err := json.Unmarshal(pingpong.msgs[i], &info[i].Msg)
		if err != nil{
			log.Fatal(err)
		}
		info[i].Msg.Receive_timestamp = pingpong.receive_timestamp[i]
		info[i].Msg.Delay = info[i].Msg.Receive_timestamp - info[i].Msg.Sent_time
	}
	out, err := json.Marshal(info)
	if err != nil{
		log.Fatal(err)
	}
	file, err := os.Create(pingpong.filename)
	if err != nil{
		log.Fatal(err)
	}
	defer file.Close()
	_, err = file.Write([]byte(out))
	if err != nil{
		log.Fatal(err)
	}
}

func (pingpong TestPingPong) receive() bool{
	count := *pingpong.n_received - *pingpong.rec_before
        *pingpong.rec_before = *pingpong.n_received
        return count > 0
}

func (pingpong TestPingPong) publish(id int, size int) {
	var str string = strings.Repeat("a", size)
	var msg msg
	msg.Sent_time = time.Now().UnixNano()
	msg.Id = id
	msg.Msg = str
	out, err := json.Marshal(msg)
        if err != nil {
                log.Fatal(err)
        }
	err = pingpong.writer.Publish(pingpong.topic1, out)
        if err != nil {
                log.Fatal(err)
        }
}

func (pingpong TestPingPong) Close(){
	pingpong.writer.Stop()
	pingpong.reader.Stop()
}
