import os
from os.path import isfile, isdir, join
import json
import numpy as np
import matplotlib.pyplot as plt

count_qmsgs = 0


def get_resfiles(test_n, subtest=False):
    directory = f'test_{test_n}/results'
    dirs = [join(directory, d) + '/data' for d in os.listdir(directory)
            if isdir(join(directory, d))]
    if subtest:
        ldirs = [[join(direct, d) for d in os.listdir(direct)
                  if isdir(join(direct, d))] for direct in dirs]
        res = []
        for dirs in ldirs:
            subres = []
            for d in dirs:
                subres.append([join(d, f) for f in os.listdir(d)
                               if isfile(join(d, f))])
            res.append(subres)
        return res
    res = []
    for direct in dirs:
        res.append([join(direct, f) for f in os.listdir(direct)
                if isfile(join(direct, f)) and f.endswith('.json')])
    return res


def get_grouped_filenames(filenames):
    res = []
    for filename in filenames[0]:
        res.append([filename])
    for k in range(0, len(res)):
        filename = res[k][0]
        for i in range(1, len(filenames)):
            for f in filenames[i]:
                if f[f.rfind('/')+1:] == filename[filename.rfind('/')+1:]:
                    res[k].append(f)
    print(res)
    return res

def from_several_jsons(filenames):
    res = []
    for f in filenames:
        if f.endswith('_sub.json'):
            res.append(sub_from_json(f))
    delay = np.mean([r[3] for r in res], axis=0)
    return delay


def sub_from_json(filename, isPingPong=False):
    with open(filename, 'r') as f:
        data = json.load(f)
    ids = [msg["msg"]["id"] for msg in data]
    send_time = [msg["msg"]["sent_time"] for msg in data]
    rec_time = [msg["msg"]["recieve_timestamp"] for msg in data]
    delay = [msg["msg"]["delay"] for msg in data]
    if not isPingPong:
        read_proc_time = [msg["msg"]["read_proc_time"] for msg in data]
    else:
        read_proc_time = []
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
    return ([d / scale for d in delays], unit, scale)


def plot_boxes(data, positions, xlabel, unit, title, plot_filename):
    widths = [(positions[i]-positions[i-1]) * 2/5
              for i in range(1, len(positions))]
    widths.append(widths[-1])
    plt.boxplot(data, positions=positions, widths=widths, whis=[0,100])
    plt.xlim([0, positions[-1] + widths[-1]])
    plt.xticks(positions)
    plt.ylabel(f'time, {unit}')
    plt.xlabel(xlabel)
    plt.title(title)
    plt.savefig(plot_filename)
    plt.clf()


def plot_graph(ids, y, unit, title, plot_filename, labels=None):
    if labels != None:
        for i in range(0, len(ids)):
            plt.plot(ids[i], y[i])
        plt.legend(labels)
    else:
        plt.plot(ids, y)
    plt.ylabel(f'time, {unit}')
    plt.xlabel('message number')
    plt.title(title)
    plt.savefig(plot_filename)
    plt.clf()


def plot_message_queue(list_counts, plot_filename):
    plt.plot([i for i in range(1, len(list_counts)+1)], list_counts)
    plt.ylabel('messages in queue')
    plt.xlabel('message number')
    plt.title('Message queue')
    plt.savefig(plot_filename)
    plt.clf()


def plot_pub_results(filenames, directory, res_name):
    try:
        os.makedirs(directory)
    except OSError:
        None
    for filename in filenames:
        directory = filename[:filename.rfind('data/')] + 'plots/'
        (proc_time, ids) = pub_from_json(filename)
        (proc_time, unit, _) = scale_values(proc_time)
        boxes = []
        for i in range(0, 10):
            k = int(len(proc_time) * (i+1)/10)
            boxes.append(np.array(proc_time[0:k]))
        plot_boxes(boxes, [len(t) for t in boxes], 'number of messages', unit, 
               'Writing time boxes', f'{directory}{res_name}_proc_time_box.png')
        plot_graph(ids, proc_time, unit, 'Writing time', f'{directory}{res_name}_proc_time.png')


def plot_sub_results(filenames, direct, res_name, isMultisub=False, isPingPong=False):
    try:
        os.makedirs(direct)
    except OSError:
        None
    if not isMultisub:
        mscale = 1
        munit = 'nsec'
        saved_delay = []
        saved_ids = []
        labels = []
        if isPingPong:
            for files in filenames:
                delay = []
                if files[0].endswith('_sub.json'):
                    buf = files[0]
                    files[0] = files[1]
                    files[1] = buf
                for i, filename in enumerate(files):
                    (send_time, receive_time,
                     _, delay_time, ids) = sub_from_json(filename, isPingPong)

                    if i == 0:
                        delay = delay_time.copy()
                    else:
                        for j, d in enumerate(delay_time):
                            delay.insert(2*j+1, d)
                (_, unit, scale) = scale_values(delay)
                if scale > mscale:
                    mscale = scale
                    munit = unit
                saved_delay.append(delay)
                saved_ids.append(list(range(0, len(delay))))
                node_name = files[0][:filename.rfind('/data/')]
                node_name = node_name[node_name.rfind('/')+1:]
                labels.append(node_name)
        else:
            for filename in filenames:
                directory = filename[:filename.rfind('data/')] + 'plots/'
                node_name = filename[:filename.rfind('/data/')]
                node_name = node_name[node_name.rfind('/')+1:]
                labels.append(node_name)

                (send_time, receive_time,
                 read_proc_time, 
                 delay_time, ids) = sub_from_json(filename, isPingPong)

                saved_delay.append(delay_time)
                saved_ids.append(ids)

                (delay_time, unit, scale) = scale_values(delay_time)
                if scale > mscale:
                    mscale = scale
                    munit = unit
                (read_proc_time, runit, _) = scale_values(read_proc_time)
                plot_graph(ids, read_proc_time, runit, 'Reading time',
                             f'{directory}{res_name}_read_proc_time.png')
                proc_time = []
                for i in range(0, 10):
                    k = int(len(read_proc_time) * (i+1)/10)
                    proc_time.append(read_proc_time[0:k])
                plot_boxes(proc_time, [len(d) for d in proc_time], 
                        'number of messages', runit, 
                        f'{node_name}: Reading time boxes', 
                        f'{directory}{node_name}_{res_name}_read_proc_time_box.png')
                list_counts = queue_size(send_time, receive_time)
                plot_message_queue(list_counts, 
                                f'{directory}{node_name}_{res_name}_queue.png')
                plot_graph(ids, delay_time, unit, f'{node_name}: Delay time', 
                           f'{directory}{node_name}_{res_name}_delay.png')
                delay = []
                for i in range(0, 10):
                    k = int(len(delay_time) * (i+1)/10)
                    delay.append(delay_time[0:k])
                plot_boxes(delay, [len(d) for d in delay],'number of messages', 
                       unit, f'{node_name}: Delay time boxes', 
                       f'{directory}{node_name}_{res_name}_delay_box.png')
        saved_delay = [[d/mscale for d in saved_delay[i]] 
                        for i in range(0, len(saved_delay))]
        plot_graph(saved_ids, saved_delay, munit, 'Delay time', 
                   f'{direct}{res_name}_delay.png', labels)
        mean = [np.mean(d) for d in saved_delay]
        plot_graph(saved_ids, [[abs(mean[i] - d) for d in saved_delay[i]] 
                   for i in range(0, len(mean))], munit, 
                   f'Jitter', f'{direct}{res_name}_jitter.png', labels)
    else:
        delay = []
        for files in filenames:
            delay.append(from_several_jsons(files))
        (delay, unit, _) = scale_values(delay)
        plot_boxes(delay, [i for i in range(1, len(delay)+1)], 
                   'count of subscribers', unit, 
                   'Delay time with multiple subscribers',
                   f'{direct}{res_name}_delay_box.png')


def plot_results(filenames, multisub=False, isPingPong=False):
    if multisub:
        filenames = filenames[0]
        directory = filenames[0][0][:filenames[0][0].rfind('data/')] + 'plots/'
        res_name = 'multisub'
        try:
            os.makedirs(directory)
        except OSError:
            None
        plot_sub_results(filenames, directory, res_name, True)
        return

    if not isPingPong:
        directory = filenames[0][:filenames[0].rfind('results/')]
        res_name = filenames[0][filenames[0].rfind('/')+1:filenames[0].rfind('_')]
    else:
        directory = filenames[0][0][:filenames[0][0].rfind('results/')]
        res_name = 'pingpong'

    try:
        os.makedirs(directory)
    except OSError:
        None
    if isPingPong:
        plot_sub_results(filenames, directory, res_name, False, isPingPong)
    elif filenames[0].endswith('_pub.json'):
        plot_pub_results(filenames, directory, res_name)
    else:
        plot_sub_results(filenames, directory, res_name)


if __name__ == '__main__':
    for i in range(0, 9):
        try:
            resfiles = get_resfiles(i, i == 3)
            if i == 3:
                for filenames in resfiles:
                    plot_results([filenames], i == 3, i == 8)
            elif i != 8:
                resfiles = get_grouped_filenames(resfiles)
                for files in resfiles:
                    plot_results(files, i == 3, i == 8)
            else:
                plot_results(resfiles, i == 3, i == 8)

        except OSError:
            continue
