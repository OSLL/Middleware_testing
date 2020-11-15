import os
from os.path import isfile, isdir, join
import json
import statistics as stat

def calc_sleep(test_n):
    directory = f'test_{test_n}/results'
    dirs = [join(directory, d) + '/data' for d in os.listdir(directory)
            if isdir(join(directory, d))]
    res = []
    for direct in dirs:
        res.extend([join(direct, f) for f in os.listdir(direct)
                if isfile(join(direct, f)) and f.endswith('.data')])
    if(not res):
        return
    filenames = [[]]
    for f in res:
        subtest = f[f.rfind('/')+1:]
        subtest = subtest[:subtest.rfind('_')]
        subtest = subtest[subtest.rfind('_')+1:]
        if(filenames[0] == []):
            filenames[0].append(f)
            continue
        for files in filenames:
            if(files[0].find(subtest) != -1):
                files.append(f)
                break
        else:
            filenames.append([f])
    print(filenames)
    json_pub = {}
    json_sub = {}
    for subtest in filenames:
        pubs_sleeps = {}
        subs_sleeps = {}
        for filename in subtest:
            node = filename[filename.rfind('/')+1:]
            node = node[:node.find('_')]
            if filename.endswith('_pub.data'):
                if node not in pubs_sleeps:
                    pubs_sleeps[node] = []
            else:
                if node not in subs_sleeps:
                    subs_sleeps[node] = []
            with open(filename,'r') as f:
                s = 'nn'
                sleep = ''
                while s:
                    s = f.readline()
                    if(s.find("nanosleep") != -1):
                        sleep = s
                    if(s.find("publish(int, unsigned int)") != -1):
                        s = sleep[sleep.rfind("="):]
                        if filename.endswith('_pub.data'):
                            pubs_sleeps.append(float(s[s.find("<")+1:s.rfind(">")]))
                        else:
                            subs_sleeps.append(float(s[s.find("<")+1:s.rfind(">")]))
                    if(s.find("receive()") != -1):
                        s = sleep[sleep.rfind("="):]
                        if filename.endswith('_pub.data'):
                            pubs_sleeps.append(float(s[s.find("<")+1:s.rfind(">")]))
                        else:
                            subs_sleeps.append(float(s[s.find("<")+1:s.rfind(">")]))
        test_name = subtest[0][subtest[0].rfind('/')+1:]
        test_name = test_name[test_name.find('_')+1:]
        test_name = test_name[:test_name.find('_')]
        pubs = {}
        subs = {}
        for key, val in pubs_sleeps.items():
            if val == []:
                val = [0]
            pubs[key] = {'sleep time': sum(val), 'median': stat.median(val)}
        for key, val in subs_sleeps.items():
            if val == []:
                val = [0]
            subs[key] = {'sleep time': sum(val), 'median': stat.median(val)}
        json_pub['Frequence: '+test_name] = pubs
        json_sub['Frequence: '+test_name] = subs
    with open(f'pubs_sleeps.json','w') as f:
        f.write(json.dumps(json_pub,indent=4, separators=(',', ': ')))
    with open(f'subs_sleeps.json','w') as f:
        f.write(json.dumps(json_sub,indent=4, separators=(',', ': ')))

if __name__ == "__main__":
    calc_sleep(3)

