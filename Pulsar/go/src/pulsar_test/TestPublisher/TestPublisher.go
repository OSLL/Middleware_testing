package TestPublisher

import(
	"strconv"
	"strings"
	"log"
	"bufio"
	"os"
	"syscall"
	"time"
	"encoding/json"
	"context"

	"github.com/apache/pulsar-client-go/pulsar"
)
type info struct{
	Msg msg_info `json:"msg"`
}

type msg_info struct{
	Id int `json:"id"`
	Proc_time int64 `json:"proc_time"`
}

type msg struct{
	Id int `json:"id"`
	Sent_time int64 `json:"sent_time"`
	Msg string `json:"msg"`
}

type TestPublisher struct{
	topic string
	msgCount int
	prior int
	cpu_index int
	min_msg_size int
	max_msg_size int
	step int
	interval int
	msgs_before_step int
	filename string
	topic_priority int
	client pulsar.Client
	producer pulsar.Producer
	write_msg_time []int64
}

func New(topic string, msgCount int, prior int, cpu_index int, min_msg_size int, max_msg_size int, step int, interval int, msgs_before_step int, filename string, topic_priority int) TestPublisher{
	pid := os.Getpid()
        if prior >= 0 {
                err := syscall.Setpriority(syscall.PRIO_PROCESS, pid, prior)
                if err != nil {
                        log.Fatal(err)
                }
        }
        if cpu_index >= 0 {
                err := os.MkdirAll("/sys/fs/cgroup/cpuset/pub_cpuset", os.ModePerm)
                if err != nil && err != os.ErrExist {
                        log.Fatal(err)
                }
                f_cpu, err := os.OpenFile("/sys/fs/cgroup/cpuset/pub_cpuset/cpuset.cpus", os.O_WRONLY, 0577)
                if err != nil {
                        log.Fatal(err)
                }
                defer f_cpu.Close()
                f_cpu.Write([]byte(strconv.Itoa(cpu_index)))

                f_exclusive, err := os.OpenFile("/sys/fs/cgroup/cpuset/pub_cpuset/cpuset.cpu_exclusive", os.O_WRONLY, 0577)
                if err != nil {
                        log.Fatal(err)
                }
                defer f_exclusive.Close()
                f_exclusive.Write([]byte("1"))

                f_mem, err := os.OpenFile("/sys/fs/cgroup/cpuset/pub_cpuset/cpuset.mems", os.O_WRONLY, 0577)
                if err != nil {
                        log.Fatal(err)
                }
                defer f_mem.Close()
                f_mem.Write([]byte("0"))

                f_task, err := os.OpenFile("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", os.O_WRONLY, 0577)
                if err != nil {
                        log.Fatal(err)
                }
                defer f_task.Close()
                f_task.Write([]byte(strconv.Itoa(pid)))
        }

	client, err := pulsar.NewClient(pulsar.ClientOptions{
            URL:               "pulsar://localhost:6650",
            OperationTimeout:  30 * time.Second,
            ConnectionTimeout: 30 * time.Second,
        })
        if err != nil {
            log.Fatal(err)
        }

	producer, err := client.CreateProducer(pulsar.ProducerOptions{
            Topic: "non-persistent://public/default/" + topic,
        })

	pub := TestPublisher{topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step, filename, topic_priority, client, producer, make([]int64, msgCount, msgCount)}
	return pub
}

func (pub TestPublisher) StartTest() int {
	time.Sleep(4*time.Second)
	cur_size := pub.min_msg_size
	for i := 0; i<pub.msgCount; i++ {
		if(i % (pub.msgs_before_step-1) == 0 && cur_size <= pub.max_msg_size){
			cur_size += pub.step;
		}
		pub.write_msg_time[i] = pub.publish(i, cur_size)

		time.Sleep(time.Duration(pub.interval) * time.Millisecond)
	}
	reader := bufio.NewReader(os.Stdin)
	end, _ := reader.ReadString('\n')
	if strings.Compare(end, "end\n") == 0 {
		pub.toJson();
		return 0;
	}
	return -1;
}

func (pub TestPublisher) toJson(){
	n := len(pub.write_msg_time)
	info := make([]info, n, n)
	for i := 0; i<n; i++{
		info[i].Msg.Id = i
		info[i].Msg.Proc_time = pub.write_msg_time[i]
	}
	out, err := json.Marshal(info)
	if err != nil{
		log.Fatal(err)
	}
	file, err := os.Create(pub.filename)
	if err != nil{
		log.Fatal(err)
	}
	defer file.Close()
	_, err = file.Write([]byte(out))
	if err != nil{
		log.Fatal(err)
	}
}

func (pub TestPublisher) publish(id int, size int) int64{
	var str string = strings.Repeat("a", size)
	var msg msg
	msg.Sent_time = time.Now().UnixNano()
	msg.Id = id
	msg.Msg = str
	out, err := json.Marshal(msg)
        if err != nil {
                log.Fatal(err)
        }
	proc_time := time.Now().UnixNano()
	_, err = pub.producer.Send(context.Background(), &pulsar.ProducerMessage{
             Payload: out,
        })
	proc_time = time.Now().UnixNano() - proc_time
        if err != nil {
                log.Fatal(err)
        }
	return proc_time
}

func (pub TestPublisher) Close(){
	pub.client.Close()
	pub.producer.Close()
}
