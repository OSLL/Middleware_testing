import subprocess
import os
import json


def write_result_to_file(node, directory, trace_to, trace_from, name_str):
    resfile = name_str + '_trace.json'
    direct = directory + resfile
    direct = direct[:direct.rfind('/')]
    try:
        os.makedirs(direct)
    except OSError:
        None
    count_to = 0
    count_from = 0
    for pid in node[1]:
        if int(pid) in trace_to.keys() or int(pid) in trace_from.keys():
            count_to += trace_to[int(pid)]
            count_from += trace_from[int(pid)]
    with open(directory + resfile, 'w') as f:
        json.dump({"copy_to_user": count_to, "copy_from_user": count_from}, f)


class CopyingTracer:

    def __init__(self):
        self.tracer = subprocess.Popen('exec staprun -R copying.ko', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd='./')

    def close(self):
        self.tracer.terminate()

    def write_results(self, sub, pub, directory):
        trace_to = {}
        trace_from = {}
        for line in self.tracer.stdout.readlines():
            s = line.decode('utf-8')
            try:
                pid = int(s[:s.find(' ')])
            except ValueError:
                continue
            if s[s.find(' ')+1] == 't':
                if pid not in trace_to:
                    trace_to[pid] = 0
                trace_to[pid] += 1
            elif s[s.find(' ')+1] == 'f':
                if pid not in trace_from:
                    trace_from[pid] = 0
                trace_from[pid] += 1

        write_result_to_file(sub, directory, trace_to, trace_from, 'first')
        write_result_to_file(pub, directory, trace_to, trace_from, 'second')
        self.tracer.stdout.close()
