import matplotlib.pyplot as plt
import os
import json

count_qmsgs = 0

def from_json(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    ids = [msg["msg"]["id"] for msg in data]
    send_time = [msg["msg"]["sent_time"] for msg in data]
    rec_time = [msg["msg"]["rec_time"] for msg in data]
    delay = [msg["msg"]["delay"] for msg in data]
    return (send_time, rec_time, delay, ids)

def from_txt(fpub, fsub):
    send_time = []
    receive_time = []
    
    send = open(fpub, 'r')
    for line in send:
        send_time.append(list(map(int, line.split())))
    send.close()

    receive = open(fsub, 'r')
    for line in receive:
        receive_time.append(list(map(int, line.split())))
    receive.close()
    return (send_time, receive_time)
    
def check_size(i,common):
    global count_qmsgs
    if common[i][1] is 'pub':
        count_qmsgs+=1
    else:
        count_qmsgs-=1
    return count_qmsgs

def queue_size(sent, recieved):
    sent_list = [(sent[i], 'pub') for i in range(0, len(sent))]
    recieved_list = [(recieved[i], 'sub') for i in range(0, len(sent))]
    common = sorted(sent_list + recieved_list, key = lambda el: el[0])
    list_ = [check_size(i, common) for i in range(0, len(common))]
    list_counts = []
    for i in range(0, len(list_)):
        if common[i][1] == 'pub':
            list_counts.append(list_[i])
    return list_counts

def plot_results(filename):
    directory = filename[:filename.rfind('/')] + '/results/'
    try:
        os.mkdir(directory)
    except OSError:
        None
    (send_time, receive_time, delay_time, ids) = from_json(filename)
    node_name = filename[filename.rfind('/')+1:filename.rfind('.')]
    list_counts = queue_size(send_time, receive_time)
    delay = []
    for i in range(0, 10):
        k = int((i+1)*len(delay_time)/10)
        delay.append(delay_time[0:k])
    delay_time = delay
    data = list(map(lambda x: [x[i]/1000000 for i in range(0, len(x))], delay_time))
    plt.boxplot(data, positions=[len(d) for d in delay], widths=[len(delay_time[-1])/25 for i in range(0, 10)], whis=[0,100])
    plt.ylabel('time, ms')
    plt.xlabel('count of messages')
    plt.savefig(directory + node_name + '_box.png')
    plt.clf()
    plt.plot(ids, list(map(lambda x: x/1000000, delay[-1])))
    plt.ylabel('time, ms')
    plt.xlabel('number of message')
    plt.savefig(directory + node_name + '_delay.png')
    plt.clf()
    plt.plot([i for i in range(1, len(list_counts)+1)], list_counts)
    plt.ylabel('messages in queue')
    plt.xlabel('number of sent messages')
    plt.savefig(directory + node_name + '_queue.png')
    plt.clf()
    count_qmsgs = 0

