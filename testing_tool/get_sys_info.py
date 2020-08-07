import platform
import cpuinfo
import json
import os
import subprocess
import psutil
import time

def get_ram_info():
    p = subprocess.Popen(args=['dmidecode', '-t', '17'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = str(p.stdout.read().decode('utf-8')).split('\n')
    res = {}
    number=0
    for line in out:
        if line.find('Permission denied') != -1:
            return {'RAM' : 'Permission denied'}
        if line.find('Memory Device') != -1:
            number+=1
        if line.find('Speed') == -1 and line.find('Size') == -1:
            continue
        lst = line.split(': ')
        res['RAM'+str(number)+' '+lst[0][1:]]=lst[1]
    return res

def scale_ram(ram):
    scales = ['B', 'KB', 'MB', 'GB']
    index = 0
    while index < (len(scales) - 1) and ram/1024 >= 1:
        ram/=1024
        index+=1
    return str(ram) + scales[index];

def max_ram(val):
    max_ram = start_ram = psutil.virtual_memory().used
    while True:
        ram = psutil.virtual_memory().used
        if ram > max_ram:
            max_ram = ram
            val.value = max_ram - start_ram
        time.sleep(1/1000)

class system:
    sys_platform = platform.platform()
    cpu = cpuinfo.get_cpu_info()
    ram = get_ram_info()
    ram_tests = {}

    def get_info(self):
        res = {"System": self.sys_platform}
        res.update(self.cpu)
        res.update(self.ram)
        res.update(self.ram_tests)
        return json.dumps(res,indent=4, separators=(',', ': '))




