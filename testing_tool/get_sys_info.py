import platform
import cpuinfo
import json
import os
import subprocess
import psutil
import time
from multiprocessing import Process, Value
import shlex

def get_commands():
    # for files use absolute path
    cmd = {}
    cmd['Qpid'] = 'qpidd -p 25565 --tcp-nodelay --max-connections 0 --ha-flow-messages 0 --session-max-unacked 10000 --default-queue-limit 0'
    return cmd

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
    losses = {}
    cmd = get_commands()
    proc = None

    def get_info(self):
        res = {"System": self.sys_platform}
        res.update(self.cpu)
        res.update(self.ram)
        res.update(self.ram_tests)
        res.update(self.losses)
        return json.dumps(res,indent=4, separators=(',', ': '))

    def start(self, framework):
        self.val = Value('i',0)
        self.check_ram = Process(target=max_ram, args=(self.val,))
        self.check_ram.start()
        self.framework = framework
        if not self.cmd.get(framework):
            print('No such command for the framework')
            return
        self.proc = subprocess.Popen(shlex.split(self.cmd[framework]), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False, close_fds=True)
        time.sleep(2)

    def end(self, test_n):
        if self.proc != None:
            self.proc.terminate()
        time.sleep(2)
        self.check_ram.terminate()
        self.ram_tests[self.framework + ' Max RAM test_' + str(test_n)] = scale_ram(self.val.value)

    def packet_loss(self, resfiles, test_n, isPingPong=False):
        packets = {}
        for filenames in resfiles:
            if isPingPong and test_n == 7:
                node_name = filenames[0][0][:filenames[0][0].rfind('/data/')]
            else:
                node_name = filenames[0][:filenames[0].rfind('/data/')]
            node_name = node_name[node_name.rfind('/')+1:]
            packets[node_name] = [0, 0]
            for filename in filenames:
                if not isPingPong and filename.find('sub') == -1:
                    continue
                if isPingPong and test_n == 7:
                    for f in filename:
                        with open(f, 'r') as f:
                            data = json.load(f)
                        send_time = [msg["msg"]["sent_time"] for msg in data]
                        packets[node_name][0] += send_time.count(0)
                        packets[node_name][1] += len(send_time)
                else:
                    with open(filename, 'r') as f:
                        data = json.load(f)
                    send_time = [msg["msg"]["sent_time"] for msg in data]
                    packets[node_name][0] += send_time.count(0)
                    packets[node_name][1] += len(send_time)
        for framework, loss in packets.items():
            self.losses[framework + ' packet loss test_' + str(test_n)] = str(loss[0] / loss[1] * 100) + '%'


