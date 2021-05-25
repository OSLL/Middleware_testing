import json
from plotting import plot_graph, scale_values, get_resfiles, get_grouped_filenames

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
    direct = direct[:direct.find('/')]
    for files in filenames:
        print('Files:',files)
        (_id, _time, _name) = plot_round_trip_time(files)
        ids.append(_id)
        round_trips.append(_time)
        labels.append(_name)
        count += 1
        if(count == 3):
            count = 0
            munit = 'nsec'
            mscale = 1
            for times in round_trips:
                (_, unit, scale) = scale_values(times)
                if scale > mscale:
                    mscale = scale
                    munit = unit
            for times in round_trips:
                times = [t/mscale for t in times]
            resfile = '_'.join(labels)
            plot_graph(ids, round_trips, munit, f'round_trip_time', f'{direct}/{resfile}_round_trip_time.png', labels)
            labels.clear()
            round_trips.clear()
            ids.clear()
    if count != 0:
        munit = 'nsec'
        mscale = 1
        for times in round_trips:
            (_, unit, scale) = scale_values(times)
            if scale > mscale:
                mscale = scale
                munit = unit
        for times in round_trips:
            times = [t/mscale for t in times]
        resfile = '_'.join(labels)
        plot_graph(ids, round_trips, munit, f'round_trip_time', f'{direct}/{resfile}_round_trip_time.png', labels)


if __name__ == "__main__":
    files = get_resfiles(6)
    print(files)
    round_trip_grouped(files)

