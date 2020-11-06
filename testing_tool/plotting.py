import os
from os.path import isfile, isdir, join
import json
import numpy as np
import matplotlib.pyplot as plt
import math

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
            buf_id = []
            buf_y = []
            for j in range(0, len(y[i])):
                if y[i][j] != 0:
                    buf_id.append(ids[i][j])
                    buf_y.append(y[i][j])
            y[i] = buf_y
            ids[i] = buf_id
            plt.plot(ids[i], y[i])
        plt.legend(labels)
    else:
        buf_id = []
        buf_y = []
        for j in range(0, len(y)):
            if y[j] != 0:
                buf_id.append(ids[j])
                buf_y.append(y[j])
        y = buf_y
        ids = buf_id
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
    for filename in filenames:
        directory = filename[:filename.rfind('data/')] + 'plots/'
        try:
            os.mkdir(directory)
        except OSError:
            None
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
    if not isMultisub:
        saved = []
        if isPingPong:
            for files in filenames:
                directory = files[0][:files[0].rfind('data/')] + 'plots/'
                try:
                    os.mkdir(directory)
                except OSError:
                    None
                delay = []
                if files[0].endswith('_sub.json'):
                    buf = files[0]
                    files[0] = files[1]
                    files[1] = buf
                for i, filename in enumerate(files):
                    (send_time, receive_time,
                     _, delay_time, ids) = sub_from_json(filename, isPingPong)
                
                    node_name = filename[:filename.rfind('/data/')]
                    node_name = node_name[node_name.rfind('/')+1:]
                    node = filename[filename.rfind('/'):filename.rfind('.json')]
                
                    list_counts = queue_size(send_time, receive_time)
                    plot_message_queue(list_counts, 
                        f'{directory}{node_name}_{res_name}_{node}_queue.png')

                    if i == 0:
                        delay = delay_time.copy()
                    else:
                        for j, d in enumerate(delay_time):
                            delay.insert(2*j+1, d)
                (_, unit, scale) = scale_values(delay)
                node_name = files[0][:filename.rfind('/data/')]
                node_name = node_name[node_name.rfind('/')+1:]

                ids = list(range(0, len(delay)))
                plot_graph(ids, [d/scale for d in delay], unit, 
                           f'{node_name}: Delay time', 
                           f'{directory}{node_name}_{res_name}_delay.png')
                delay_time = []
                for i in range(0, 10):
                    k = int(len(delay) * (i+1)/10)
                    delay_time.append(delay[0:k])
                plot_boxes(delay_time, [len(d) for d in delay_time],
                       'number of messages', 
                       unit, f'{node_name}: Delay time boxes', 
                       f'{directory}{node_name}_{res_name}_delay_box.png')
                mean = np.mean(delay)
                plot_graph(ids, [abs(mean - d) for d in delay], 
                           unit, f'Jitter', 
                           f'{directory}{node_name}_{res_name}_jitter.png')
                saved.append((ids, delay, node_name, scale, unit))
        else:
            for filename in filenames:
                directory = filename[:filename.rfind('data/')] + 'plots/'
                try:
                    os.mkdir(directory)
                except OSError:
                    None
                node_name = filename[:filename.rfind('/data/')]
                node_name = node_name[node_name.rfind('/')+1:]

                (send_time, receive_time,
                 read_proc_time, 
                 delay_time, ids) = sub_from_json(filename, isPingPong)

                saved_delay_time = delay_time
                (delay_time, unit, scale) = scale_values(delay_time)
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
                mean = np.mean(delay_time)
                plot_graph(ids, [abs(mean - d) 
                           for d in delay_time], unit, f'Jitter', 
                           f'{directory}{node_name}_{res_name}_jitter.png')
                saved.append((ids, saved_delay_time, node_name, scale, unit))
        saved.sort(key=lambda x: x[3])
        for i in range(0, math.ceil(len(saved)/3.0)):
            saved_ids = list(map(lambda x: x[0], saved))[3*i:3*i+3]
            saved_delay = list(map(lambda x: x[1], saved))[3*i:3*i+3]
            labels = list(map(lambda x: x[2], saved))[3*i:3*i+3]
            scales = list(map(lambda x: x[3], saved))[3*i:3*i+3]
            units = list(map(lambda x: x[4], saved))[3*i:3*i+3]

            mscale = max(scales)
            index = scales.index(mscale)
            munit = units[index]
            node_names_prefix = '_'.join(labels)

            saved_delay = [[d/mscale for d in saved_delay[i]]
                            for i in range(0, len(saved_delay))]
            plot_graph(saved_ids, saved_delay, munit, 'Delay time', 
                       f'{direct}{node_names_prefix}_{res_name}_delay.png', 
                       labels)
            mean = [np.mean(d) for d in saved_delay]
            plot_graph(saved_ids, [[abs(mean[i] - d) for d in saved_delay[i]] 
                       for i in range(0, len(mean))], munit, f'Jitter', 
                       f'{direct}{node_names_prefix}_{res_name}_jitter.png', 
                       labels)
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
        
        try:
            os.makedirs(directory)
        except OSError:
            None

        if not isPingPong:
            res_name = 'multisub'
            plot_sub_results(filenames, directory, res_name, True)
        else:
            for f in filenames:
                res_name = f[f.rfind('/'):f.rfind('.json')]
                plot_sub_results(f, directory, res_name)
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
    for i in range(1, 8):
        try:
            resfiles = get_resfiles(i, i == 2)
            if i == 2:
                for filenames in resfiles:
                    plot_results([filenames], i == 2, i > 5)
            elif i < 6:
                resfiles = get_grouped_filenames(resfiles)
                for files in resfiles:
                    plot_results(files, i == 2, i > 5)
            else:
                plot_results(resfiles, i == 2, i > 5)

        except OSError:
            continue
