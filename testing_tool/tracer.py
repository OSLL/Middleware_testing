import subprocess
import os

class CopyingTracer:

    def __init__(self):
        self.tracer = subprocess.Popen('exec staprun -R copying.ko', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd='./')

    def close(self):
        self.tracer.terminate()

    def write_results(self, subs, pubs, directory):
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
        for s in subs:
            config = s[1]
            resfile = config[:config.find('.json')] + '_sub.trace'
            direct = directory + resfile
            direct = direct[:direct.rfind('/')]
            try:
                os.makedirs(direct)
            except OSError:
                None
            with open(directory + resfile, 'w') as f:
                f.write(str(trace_to[s[0].pid]) + ' ' + str(trace_from[s[0].pid]))
        for p in pubs:
            config = p[1]
            resfile = config[:config.find('.json')] + '_pub.trace'
            direct = directory + resfile
            direct = direct[:direct.rfind('/')]
            try:
                os.makedirs(direct)
            except OSError:
                None
            with open(directory + resfile, 'w') as f:
                f.write(str(trace_to[p[0].pid]) + ' ' + str(trace_from[p[0].pid]))
        self.tracer.stdout.close()
