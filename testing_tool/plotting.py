import os
import json
import numpy as np
import matplotlib.pyplot as plt

count_qmsgs = 0


def from_several_jsons(filenames):
    res = []
    for f in filenames:
        if f.endswith('_sub.json'):
            res.append(sub_from_json(f))
    delay = np.mean([r[3] for r in res], axis=0)
    return delay


def sub_from_json(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    ids = [msg["msg"]["id"] for msg in data]
    send_time = [msg["msg"]["sent_time"] for msg in data]
    rec_time = [msg["msg"]["recieve_timestamp"] for msg in data]
    delay = [msg["msg"]["delay"] for msg in data]
    read_proc_time = [msg["msg"]["read_proc_time"] for msg in data]
    return (send_time, rec_time, read_proc_time, delay, ids)


def pub_from_json(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    ids = [msg["msg"]["id"] for msg in data]
    proc_time = [msg["msg"]["proc_time"] for msg in data]
    return (proc_time, ids)


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
    count_qmsgs = 0
    list_counts = []
    for i in range(0, len(list_)):
        if common[i][1] == 'pub':
            list_counts.append(list_[i])
    return list_counts


def scale_values(delays):
    if isinstance(delays[0], np.ndarray):
        maximum = max(map(max, delays))
    else:
        maximum = max(delays)
    if (maximum / 1000000000) >= 1:
        scale = 1000000000
        unit = 'sec'
    elif (maximum / 1000000) >= 1:
        scale = 1000000
        unit = 'msec'
    elif (maximum / 1000) >= 1:
        scale = 1000
        unit = 'usec'
    else:
        scale = 1
        unit = 'nsec'
    return ([d / scale for d in delays], unit)


def plot_boxes(data, positions, xlabel, unit, plot_filename):
    widths = [(positions[i]-positions[i-1]) * 2/5
              for i in range(1, len(positions))]
    widths.append(widths[-1])
    plt.boxplot(data, positions=positions, widths=widths, whis=[0,100])
    plt.xlim([0, positions[-1] + widths[-1]])
    plt.xticks(positions)
    plt.ylabel('time, ' + unit)
    plt.xlabel(xlabel)
    plt.savefig(plot_filename)
    plt.clf()


def plot_graph(ids, y, unit, plot_filename):
    plt.plot(ids, y)
    plt.ylabel('time, ' + unit)
    plt.xlabel('number of message')
    plt.savefig(plot_filename)
    plt.clf()


def plot_message_queue(list_counts, plot_filename):
    plt.plot([i for i in range(1, len(list_counts)+1)], list_counts)
    plt.ylabel('messages in queue')
    plt.xlabel('number of message')
    plt.savefig(plot_filename)
    plt.clf()


def plot_pub_results(filename, directory, res_name):
    (proc_time, ids) = pub_from_json(filename)
    (proc_time, unit) = scale_values(proc_time)
    boxes = []
    for i in range(0, 10):
        k = int(len(proc_time) * (i+1)/10)
        boxes.append(np.array(proc_time[0:k]))
    plot_boxes(boxes, [len(t) for t in boxes], 'count of messages', unit, 
               directory + res_name + '_proc_time_box.png')
    plot_graph(ids, proc_time, unit, directory + res_name + '_proc_time.png')


def plot_sub_results(filenames, directory, res_name):
    if len(filenames) == 1:
        (send_time, receive_time,
         read_proc_time, delay_time, ids) = sub_from_json(filenames[0])

        (delay_time, unit) = scale_values(delay_time)
        (read_proc_time, runit) = scale_values(read_proc_time)
        list_counts = queue_size(send_time, receive_time)
        delay = []
        proc_time = []
        for i in range(0, 10):
            k = int(len(delay_time) * (i+1)/10)
            delay.append(delay_time[0:k])
            k = int(len(read_proc_time) * (i+1)/10)
            proc_time.append(read_proc_time[0:k])
        plot_message_queue(list_counts, directory + res_name + '_queue.png')
        plot_graph(ids, delay[-1], unit, directory + res_name + '_delay.png')
        plot_boxes(delay, [len(d) for d in delay], 'count of messages', unit,
                   directory + res_name + '_delay_box.png')
        plot_graph(ids, read_proc_time, runit, 
                   directory + res_name + '_read_proc_time.png')
        plot_boxes(proc_time, [len(d) for d in proc_time], 'count of messages', 
                   runit, directory + res_name + '_read_proc_time_box.png')
    else:
        delay = []
        for files in filenames:
            delay.append(from_several_jsons(files))
        (delay, unit) = scale_values(delay)
        plot_boxes(delay, [i for i in range(1, len(delay)+1)], 
                   'count of messages', unit, 
                   directory + res_name + '_delay_box.png')


def plot_results(filenames):
    if isinstance(filenames[0], list):
        filenames = filenames[0]
        directory = filenames[0][0][:filenames[0][0].rfind('data/')] + 'plots/'
        res_name = 'multisub'
        try:
            os.mkdir(directory)
        except OSError:
            None
        plot_sub_results(filenames, directory, res_name)
        return
    directory = filenames[0][:filenames[0].rfind('data/')] + 'plots/'
    try:
        os.mkdir(directory)
    except OSError:
        None
    res_name = filenames[0][filenames[0].rfind('/')+1:filenames[0].rfind('_')]
    if filenames[0].endswith('_pub.json'):
        plot_pub_results(filenames[0], directory, res_name)
    else:
        plot_sub_results(filenames, directory, res_name)


