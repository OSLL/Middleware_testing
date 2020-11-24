import json
import numpy as np
import matplotlib.pyplot as plt
from plotting import plot_graph, scale_values, get_resfiles, get_grouped_filenames

def plot_write_read_jitter(resfiles, isMultiSub=False, isPingPong=False):
    if isMultiSub:
        nothing = []
    elif isPingPong:
        nothing = []
    else:
        grouped = get_grouped_filenames(resfiles)
        for filenames in grouped:
            direct = filenames[0][:filenames[0].rfind('results/')]
            res_name = filenames[0][filenames[0].rfind('/')+1:filenames[0].rfind('_')]
            labels = []
            proc_times = []
            all_ids = []
            mscale = 1
            munit = 'nsec'
            if(filenames[0].endswith('_pub.json')):
                key = 'proc_time'
            else:
                key = 'read_proc_time'
            for i, filename in enumerate(filenames):
                if(i%3 == 0 and i != 0):
                    names = '_'.join(labels)
                    plot_graph(all_ids, proc_times, munit, f'{key}_jitter', f'{direct}/{names}_{res_name}_{key}_jitter.png', labels) 
                    labels.clear()
                    proc_times.clear()
                    all_ids.clear()
                    mscale = 1
                    munit = 'nsec'
                node_name = filename[:filename.rfind('/data/')]
                node_name = node_name[node_name.rfind('/')+1:]
                labels.append(node_name)
                with open(filename, 'r') as f:
                    data = json.load(f)
                proc_time = [msg["msg"][key] for msg in data]
                mean = np.mean(proc_time)
                (proc_time, unit, scale) = scale_values([abs(mean - t) for t in proc_time])
                if scale > mscale:
                    mscale = scale
                    munit = unit
                ids = [msg["msg"]["id"] for msg in data]
                all_ids.append(ids)
                proc_times.append(proc_time)
            if labels != []:
                plot_graph(all_ids, proc_times, munit, f'{key}_jitter', f'{direct}/{res_name}_{key}_jitter.png', labels) 

if __name__ == '__main__':
    for i in range(0,9):
        try:
            resfiles = get_resfiles(i, i == 3)
            plot_write_read_jitter(resfiles, i == 3, i == 8)
        except OSError:
            continue
