import os
from os.path import isfile, isdir, join
import json
import numpy as np
import matplotlib.pyplot as plt

def get_files():
    directory = 'results'
    dirs = [join(directory, d) + '/data' for d in os.listdir(directory)
            if isdir(join(directory, d))]
    res=[]
    for direct in dirs:
        res += [join(direct, f) for f in os.listdir(direct)
                if isfile(join(direct, f)) and f.endswith('.json')]
    return res

def plot_graphs(ids, delays, unit, title, filename, labels):
    for i in range(len(ids)):
        plt.plot(ids[i],delays[i],label=labels[i])
    plt.legend()
    plt.ylabel(f'time, {unit}')
    plt.xlabel('message number')
    plt.title(title)
    plt.savefig(filename)
    plt.clf()


def scale_values(delays):
    maxs=[]
    for d in delays:
        maxs.append(max(d))
    maximum = max(maxs)
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
    unit = 'msec'
    delay = np.array(delays) / scale
    return delay, unit



def plot(directory):
    files=get_files()
    id_lf=[]
    delay_lf=[]
    id_ls=[]
    delay_ls=[]
    labels=[]
    for filename in files:
        label=filename[(filename.find('/')+1):(filename.find('/',(filename.find('/')+1)))]
        if label not in labels:
            labels.append(label)
        with open(filename, 'r') as f:
            data = json.load(f)
        ids = [msg["msg"]["id"] for msg in data]
        delay = [msg["msg"]["delay"] for msg in data]
        if "first" in filename:
            id_lf.append(ids)
            delay_lf.append(delay)
        else:
            id_ls.append(ids)
            delay_ls.append(delay)
    maxs=[]
    delay_lf, unit=scale_values(delay_lf)
    plot_graphs(id_lf, delay_lf, unit, "First node delay", f"{directory}/first_delays.png",labels)
    delay_ls, unit=scale_values(delay_ls)
    plot_graphs(id_ls, delay_ls, unit, "Second node delay", f"{directory}/second_delays.png",labels)
    for df in delay_lf:
        mean=np.mean(df)
        for i in range(len(df)):
            df[i]=abs(df[i]-mean)
    for ds in delay_ls:
        mean=np.mean(ds)
        for i in range(len(ds)):
            ds[i]=abs(ds[i]-mean)
    delay_lf, unit=scale_values(delay_lf)
    plot_graphs(id_lf, delay_lf, unit, "first jitter", f"{directory}/first_jitter.png",labels)
    delay_ls, unit=scale_values(delay_ls)
    plot_graphs(id_ls, delay_ls, unit, "second_jitter", f"{directory}/second_jitter.png",labels)


if __name__ == "__main__":
    plot("results")
