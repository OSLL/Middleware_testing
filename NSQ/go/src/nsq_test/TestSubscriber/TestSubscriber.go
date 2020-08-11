package TestSubscriber

import(
	"strconv"
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
	Read_proc_time int64 `json:"read_proc_time"`
	Receive_timestamp int64 `json:"recieve_timestamp"`
	Sent_time int64 `json:"sent_time"`
	Delay int64 `json:"delay"`
}

type TestSubscriber struct{
	topic string
	msgCount int
	prior int
	cpu_index int
	max_msg_size int
	step int
	interval int
	msgs_before_step int
	filename string
	topic_priority int
	reader *nsq.Consumer
	read_msg_time []int64
	msgs [][]byte
	receive_timestamp []int64
	n_received int
	rec_before int
}

func New(topic string, msgCount int, prior int, cpu_index int, max_msg_size int, step int, interval int, msgs_before_step int, filename string, topic_priority int) TestSubscriber{
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
        reader, err := nsq.NewConsumer(topic, "ch", config)
        if err != nil {
                log.Fatal(err)
        }
	sub := TestSubscriber{topic, msgCount, prior, cpu_index, max_msg_size, step, interval, msgs_before_step, filename, topic_priority, reader, make([]int64, msgCount, msgCount), make([][]byte, msgCount, msgCount), make([]int64, msgCount, msgCount), 0, 0}
	reader.AddHandler(nsq.HandlerFunc(func(msg *nsq.Message) error {
		if sub.n_received >= sub.msgCount {
		    return nil
		}
		sub.read_msg_time[sub.n_received] = time.Now().UnixNano()
		sub.msgs[sub.n_received] = msg.Body
		sub.read_msg_time[sub.n_received] = time.Now().UnixNano() - sub.read_msg_time[sub.n_received]
		sub.receive_timestamp[sub.n_received] = time.Now().UnixNano()
		sub.n_received += 1
		return nil
	}))
	err = reader.ConnectToNSQD("127.0.0.1:4150")
        if err != nil {
            log.Fatal(err)
        }

	return sub
}

func (sub TestSubscriber) StartTest() int {
	start_timeout := time.Now().UnixNano()
	end_timeout := start_timeout
	for true {
		if sub.receive() {
			start_timeout = time.Now().UnixNano()
		} else {
			end_timeout = time.Now().UnixNano()
			if (end_timeout - start_timeout > int64(timeout)) {
				break
			}
		}
		time.Sleep(time.Millisecond)
	}
	sub.toJson();
	return 0;
}

func (sub TestSubscriber) toJson(){
	n := len(sub.msgs)
	info := make([]info, n, n)
	for i := 0; i<n; i++{
		err := json.Unmarshal(sub.msgs[i], &info[i].Msg)
		if err != nil{
			log.Fatal(err)
		}
		info[i].Msg.Read_proc_time = sub.read_msg_time[i]
		info[i].Msg.Receive_timestamp = sub.receive_timestamp[i]
		info[i].Msg.Delay = info[i].Msg.Receive_timestamp - info[i].Msg.Sent_time
	}
	out, err := json.Marshal(info)
	if err != nil{
		log.Fatal(err)
	}
	file, err := os.Create(sub.filename)
	if err != nil{
		log.Fatal(err)
	}
	defer file.Close()
	_, err = file.Write([]byte(out))
	if err != nil{
		log.Fatal(err)
	}
}

func (sub TestSubscriber) receive() bool{
	count := sub.n_received - sub.rec_before
	sub.rec_before = sub.n_received
	return count > 0
}

func (sub TestSubscriber) Close(){
	sub.reader.Stop()
}
