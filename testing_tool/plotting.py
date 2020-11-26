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
    return res

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
    if common[i][1] == 'pub':
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
    try:
        os.makedirs(plot_filename[:plot_filename.rfind('/')])
    except OSError:
        None
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
    try:
        os.makedirs(plot_filename[:plot_filename.rfind('/')])
    except OSError:
        None

    plt.plot([i for i in range(1, len(list_counts)+1)], list_counts)
    plt.ylabel('messages in queue')
    plt.xlabel('message number')
    plt.title('Message queue')
    plt.savefig(plot_filename)
    plt.clf()


def plot_pub_results(filenames, test_n):
    for filename in filenames:
        node_name = filename[:filename.rfind('/data/')]
        node_name = node_name[node_name.rfind('/') + 1:]
        node = filename[filename.rfind('/')+1:filename.rfind('.json')]
        if test_n in [1, 3, 4, 5]:
            s = '/' + node[:node.find('_')]
            node = node[node.find('_')+1:]
        else:
            s = filename[:filename.rfind('/')]
        subdir = s[s.rfind('/'):]
        if subdir == '/data':
            subdir = ''
        else:
            subdir += '/'
        directory = filename[:filename.rfind('test_')+7] + 'plots/' + node_name + '/' + subdir + '/write_time/'
        try:
            os.makedirs(directory)
        except OSError:
            None
        (proc_time, ids) = pub_from_json(filename)
        (proc_time, unit, _) = scale_values(proc_time)
        boxes = []
        for i in range(0, 10):
            k = int(len(proc_time) * (i+1)/10)
            boxes.append(np.array(proc_time[0:k]))
        plot_boxes(boxes, [len(t) for t in boxes], 'number of messages', unit, 
               'Writing time boxes', f'{directory}{node}_proc_time_box.png')
        plot_graph(ids, proc_time, unit, 'Writing time', f'{directory}{node}_proc_time.png')


def plot_sub_results(test_n, filenames, isMultisub=False, isPingPong=False, grouping=True):
    if not isMultisub:
        saved = []
        directories = []
        if isPingPong:
            delays = {}
            for files in filenames:
                if len(files) == 0:
                    continue
                saved.append([])
                directory = files[0][:files[0].rfind('test_')+7] + 'plots/'
                s = files[0][:files[0].rfind('/')]
                subdir = s[s.rfind('/'):]
                if subdir == '/data':
                    directories.append(directory)
                    subdir = ''
                else:
                    directories.append(directory + subdir + '_')
                    subdir += '/'
                try:
                    os.makedirs(directory)
                except OSError:
                    None
                delay = []
                if files[0].endswith('_sub.json') and grouping:
                    buf = files[0]
                    files[0] = files[1]
                    files[1] = buf
                for i, filename in enumerate(files):
                    (send_time, receive_time,
                     read_proc_time, delay_time, ids) = sub_from_json(filename)
                
                    node_name = filename[:filename.rfind('/data/')]
                    node_name = node_name[node_name.rfind('/')+1:]
                    node = filename[filename.rfind('/')+1:filename.rfind('.json')]

                    (read_proc_time, runit, _) = scale_values(read_proc_time)
                    plot_graph(ids, read_proc_time, runit, 'Reading time',
                               f'{directory}{node_name}{subdir}/read_time/{node}_read_proc_time.png')
                    proc_time = []
                    for i in range(0, 10):
                        k = int(len(read_proc_time) * (i + 1) / 10)
                        proc_time.append(read_proc_time[0:k])
                    plot_boxes(proc_time, [len(d) for d in proc_time],
                               'number of messages', runit,
                               f'{node_name}: Reading time boxes',
                               f'{directory}{node_name}{subdir}/read_time/{node}_read_proc_time_box.png')
                
                    list_counts = queue_size(send_time, receive_time)
                    plot_message_queue(list_counts, 
                        f'{directory}{node_name}{subdir}/queue/{node}_queue.png')

                    if not grouping:
                        _delay_time = delay_time
                        (delay_time, unit, scale) = scale_values(delay_time)
                        plot_graph(ids, delay_time, unit, 
                             f'{node_name}: Delay time', 
                             f'{directory}{node_name}{subdir}/delay/{node}_delay.png')
                        delay = []
                        for i in range(0, 10):
                            k = int(len(delay_time) * (i+1)/10)
                            delay.append(delay_time[0:k])
                        plot_boxes(delay, [len(d) for d in delay],
                             'number of messages', 
                             unit, f'{node_name}: Delay time boxes', 
                             f'{directory}{node_name}{subdir}/delay/{node}_delay_box.png')
                        mean = np.mean(delay_time)
                        plot_graph(ids, [abs(mean - d) for d in delay_time], 
                                unit, f'Jitter', 
                                f'{directory}{node_name}{subdir}/delay/{node}_jitter.png')
                        saved[-1].append((ids, _delay_time, node_name, scale, unit, node))
                    else:
                        if node_name not in delays:
                            delays[node_name] = delay.copy()
                        else:
                            if filename.endswith('_sub.json'):
                                for j, d in enumerate(delay_time):
                                    delays[node_name].insert(2*j+1, d)
                            else:
                                for j, d in enumerate(delay_time):
                                    delays[node_name].insert(2*j, d)
                if grouping:
                    for node_name in delays:
                        delay = delays[node_name]
                        (_, unit, scale) = scale_values(delay)
                        #node_name = files[0][:files[0].rfind('/data/')]
                        #node_name = node_name[node_name.rfind('/')+1:]

                        ids = list(range(0, len(delay)))
                        plot_graph(ids, [d/scale for d in delay], unit, 
                                   f'{node_name}: Delay time', 
                                   f'{directory}{node_name}{subdir}/delay/{node}_delay.png')
                        delay_time = []
                        for i in range(0, 10):
                            k = int(len(delay) * (i+1)/10)
                            delay_time.append(delay[0:k])
                        plot_boxes(delay_time, [len(d) for d in delay_time],
                               'number of messages', 
                               unit, f'{node_name}: Delay time boxes', 
                               f'{directory}{node_name}{subdir}/delay/{node}_delay_box.png')
                        mean = np.mean(delay)
                        plot_graph(ids, [abs(mean - d) for d in delay], 
                                   unit, f'Jitter', 
                                   f'{directory}{node_name}{subdir}/delay/{node}_jitter.png')
                        saved[-1].append((ids, delay, node_name, scale, unit))
        else:
            saved.append([])
            for filename in filenames:
                directory = filename[:filename.rfind('results/')] + 'plots/'
                node = filename[filename.rfind('/')+1:filename.rfind('.json')]
                if test_n in [1, 3, 4, 5]:
                    s = '/' + node[:node.find('_')]
                    node = node[node.find('_')+1:]
                else:
                    s = filename[:filename.rfind('/')]
                subdir = s[s.rfind('/'):]
                if subdir == '/data':
                    directories.append(directory)
                    subdir = ''
                else:
                    directories.append(directory + subdir + '_')
                    subdir += '/'
                try:
                    os.makedirs(directory)
                except OSError:
                    None
                
                node_name = filename[:filename.rfind('/data/')]
                node_name = node_name[node_name.rfind('/')+1:]

                (send_time, receive_time,
                 read_proc_time, 
                 delay_time, ids) = sub_from_json(filename)

                saved_delay_time = delay_time
                (delay_time, unit, scale) = scale_values(delay_time)
                (read_proc_time, runit, _) = scale_values(read_proc_time)
                plot_graph(ids, read_proc_time, runit, 'Reading time',
                             f'{directory}{node_name}{subdir}/read_time/{node}_read_proc_time.png')
                proc_time = []
                for i in range(0, 10):
                    k = int(len(read_proc_time) * (i+1)/10)
                    proc_time.append(read_proc_time[0:k])
                plot_boxes(proc_time, [len(d) for d in proc_time], 
                        'number of messages', runit, 
                        f'{node_name}: Reading time boxes', 
                        f'{directory}{node_name}{subdir}/read_time/{node}_read_proc_time_box.png')
                list_counts = queue_size(send_time, receive_time)
                plot_message_queue(list_counts, 
                                f'{directory}{node_name}{subdir}/queue/{node}_queue.png')
                plot_graph(ids, delay_time, unit, f'{node_name}: Delay time', 
                           f'{directory}{node_name}{subdir}/delay/{node}_delay.png')
                delay = []
                for i in range(0, 10):
                    k = int(len(delay_time) * (i+1)/10)
                    delay.append(delay_time[0:k])
                plot_boxes(delay, [len(d) for d in delay],'number of messages', 
                       unit, f'{node_name}: Delay time boxes', 
                       f'{directory}{node_name}{subdir}/delay/{node}_delay_box.png')
                mean = np.mean(delay_time)
                plot_graph(ids, [abs(mean - d) 
                    for d in delay_time], unit, f'{node_name}: Jitter', 
                           f'{directory}{node_name}{subdir}/delay/{node}_jitter.png')
                saved[-1].append((ids, saved_delay_time, node_name, scale, unit, node))
        pref = []
        saved_ = []
        directories_ = []
        if (isPingPong and not grouping) or not isPingPong:
            for i in range(len(saved)):
                prefixes = set([s[5] for s in saved[i]])
                for p in prefixes:
                    saved_.append(list(filter(lambda x: x[5] == p, saved[i])))
                    pref.append(p+'_')
                    directories_.append(directories[i])
        else:
            saved_ = saved
            node_pref = ''
            directories_ = directories

        for j, save in enumerate(saved_):
            directory = directories_[j]
            if (isPingPong and not grouping) or not isPingPong:
                node_pref = pref[j]
            save.sort(key=lambda x: x[3])
            for i in range(0, math.ceil(len(save)/3.0)):
                saved_ids = list(map(lambda x: x[0], save))[3*i:3*i+3]
                saved_delay = list(map(lambda x: x[1], save))[3*i:3*i+3]
                labels = list(map(lambda x: x[2], save))[3*i:3*i+3]
                scales = list(map(lambda x: x[3], save))[3*i:3*i+3]
                units = list(map(lambda x: x[4], save))[3*i:3*i+3]

                mscale = max(scales)
                index = scales.index(mscale)
                munit = units[index]
                node_names_prefix = '_'.join(labels)

                saved_delay = [[d/mscale for d in saved_delay[i]]
                                for i in range(0, len(saved_delay))]
                plot_graph(saved_ids, saved_delay, munit, 'Delay time', 
                f'{directory}{node_pref}{node_names_prefix}_delay.png', 
                          labels)
                mean = [np.mean(d) for d in saved_delay]
                plot_graph(saved_ids, [[abs(mean[i] - d) for d in saved_delay[i]] 
                           for i in range(0, len(mean))], munit, f'Jitter', 
                           f'{directory}{node_pref}{node_names_prefix}_jitter.png', 
                           labels)
    else:
        filename = filenames[0][0]
        directory = filename[:filename.rfind('results/')] + 'plots/'
        node_name = filename[:filename.rfind('/data/')]
        node_name = node_name[node_name.rfind('/')+1:]
        try:
            os.makedirs(directory + node_name)
        except OSError:
            None
        delay = []
        for files in filenames:
            delay.append(from_several_jsons(files))
        (delay, unit, _) = scale_values(delay)
        plot_boxes(delay, [i for i in range(1, len(delay)+1)], 
                    'count of subscribers', unit, 
                    'Delay time with multiple subscribers',
                    f'{directory}{node_name}/delay_box.png')




def plot_results(filenames, test_n, multisub=False, isPingPong=False, grouping=True):
    if multisub:
        filenames = filenames[0]
        
        if not isPingPong:
            plot_sub_results(test_n, filenames, True)
        else:
            for files in filenames:
                plot_pub_results(files, test_n)
            plot_sub_results(test_n, filenames, isPingPong=True, grouping=False)
        return

    if isPingPong:
        for files in filenames:
            plot_pub_results(files, test_n)
        plot_sub_results(test_n, filenames, False, isPingPong, grouping)
    elif filenames[0].endswith('_pub.json'):
        plot_pub_results(filenames, test_n)
    else:
        plot_sub_results(test_n, filenames)



def plot_round_trip_time(filenames, need_plot = False):
    with open(filenames[0], 'r') as f:
        data1 = json.load(f)
    with open(filenames[1], 'r') as f:
        data2 = json.load(f)
    sent_time1 = [msg["msg"]["sent_time"] for msg in data1]
    sent_time2 = [msg["msg"]["sent_time"] for msg in data2]
    rec_time1 = [msg["msg"]["recieve_timestamp"] for msg in data1]
    rec_time2 = [msg["msg"]["recieve_timestamp"] for msg in data2]
    ids = [[msg["msg"]["id"] for msg in data1]]
    round_trip = []
    if sent_time1[0] < sent_time2[0] and rec_time1[0] < rec_time2[0]:
        for i in range(len(sent_time1)):
            round_trip.append(rec_time2[i] - sent_time1[i])
    else:
        for i in range(len(sent_time1)):
            round_trip.append(rec_time1[i] - sent_time2[i])
    node_name = filenames[0][:filenames[0].rfind('/data')]
    node_name = node_name[node_name.rfind('/')+1:]
    if need_plot == True:
        (round_trip, unit,_) = scale_values(round_trip)
        round_trip = [round_trip]
        plot_graph(ids, round_trip, unit, f'round_trip_time', f'round_trip_time.png', [f'{node_name}']) 
    else:
        return ids[0], round_trip, node_name


def round_trip_grouped(filenames):
    ids = []
    round_trips = []
    labels = []
    count = 0
    direct = filenames[0][0][filenames[0][0].find('test_'):]
    direct = direct[:direct.find('/')+1] + 'plots/'
    try:
        os.makedirs(direct)
    except OSError:
        None
    for files in filenames:
        if len(files) == 0:
            continue
        (_id, _time, _name) = plot_round_trip_time(files)
        ids.append(_id)
        round_trips.append(_time)
        labels.append(_name)
    for i in range(0,len(labels),3):
        munit = 'nsec'
        mscale = 1
        for times in round_trips[i:i+3]:
            (_, unit, scale) = scale_values(times)
            if scale > mscale:
                mscale = scale
                munit = unit
        for times in round_trips[i:i+3]:
            times = [t/mscale for t in times]
        node_name = '_'.join(labels[i:i+3])
        plot_graph(ids[i:i+3], round_trips[i:i+3], munit, f'round_trip_time', f'{direct}{node_name}/RTT/round_trip_time.png', labels[i:i+3])


if __name__ == '__main__':
    for i in range(1, 9):
        try:
            resfiles = get_resfiles(i, i == 2 or i == 7)
            if i == 2:
                for filenames in resfiles:
                    plot_results([filenames], i, i == 2, i > 5)
            elif i < 6:
                resfiles = get_grouped_filenames(resfiles)
                for files in resfiles:
                    plot_results(files, i, i == 2, i > 5)
            else:
                if i == 6:
                    round_trip_grouped(resfiles)
                if i == 7:
                    files = []
                    for filenames in resfiles:
                        while len(filenames) > len(files):
                            files.append([])
                        for j in range(len(filenames)):
                            for f in filenames[j]:
                                files[j].append(f)
                    for filenames in files:
                        plot_results([[filenames]], i, i == 7, i > 5, grouping=(i<7))
                else:
                    files = []
                    for filenames in resfiles:
                        for filename in filenames:
                            files.append(filename)
                    plot_results([files], i, i == 7, i > 5, grouping=(i<7))
        except OSError:
            continue
