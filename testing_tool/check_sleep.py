import os
from os.path import isfile, join
import json

def calc_sleep(test_n):
    direct = f"test_{test_n}"
    filenames = [join(direct, f) for f in os.listdir(direct)
            if isfile(join(direct, f)) and f.endswith('.data')]
    if(not filenames):
        return
    res = [[]]
    for f in filenames:
        subtest = f[f.rfind('subtest'):f.rfind('_')]
        if(res[0] == []):
            res[0].append(f)
            continue
        for files in res:
            if(files[0].find(subtest) != -1):
                files.append(f)
                break
        else:
            res.append([f])
    filenames = []
    for files in res:
        k_lists = [[]]
        for f in files:
            k_number = f[f.rfind('_'):]
            if(k_lists[0] == []):
                k_lists[0].append(f)
                continue
            for name in k_lists:
                if(name[0].endswith(k_number)):
                    name.append(f)
                    break
            else:
                k_lists.append([f])
        filenames.append(k_lists)
    print(filenames)
    for files in filenames:
        for names in files:
            d = {}
            for filename in names:
                k = 0.0
                with open(filename,'r') as f:
                    s = 'nn'
                    while s:
                        s = f.readline()
                        if(s.find("nanosleep") == -1):
                            continue
                        isStartTest = f.readline()
                        isStartTest = f.readline()
                        isStartTest = f.readline()
                        if(isStartTest.find("StartTest()")!=-1):
                            continue
                        else:
                            s = s[s.rfind("="):]
                            k += float(s[s.find("<")+1:s.rfind(">")])
                node_name = filename[filename.rfind('/')+1:]
                d[node_name[:node_name.find('_')]] = k
            subtest_k = filename[filename.rfind('subtest'):filename.rfind('.data')] 
            with open(f'{direct}_{subtest_k}_sleep_times.json','w') as f:
                f.write(json.dumps(d,indent=4, separators=(',', ': ')))

if __name__ == "__main__":
    for i in range(1,8):
        calc_sleep(i)

