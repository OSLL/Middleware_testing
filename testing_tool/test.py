import unittest
import os
import subprocess
import json
import time

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../FastRTPS/test/build/FastRTPSTest publisher"]
    subs = ["../FastRTPS/test/build/FastRTPSTest subscriber"]

    def create_process(self, name, args):
        with open('args.json', 'w') as f:
            json.dump(args, f)
        return subprocess.Popen(name+' args.json', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")

    def test0(self):
        print(">>> running test0")
        try:
            os.mkdir("test0")
        except OSError:
            None
        mlen = [50, 60000]
        args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":5000, "min_msg_size":100, "max_msg_size":100,  "step":0, "msgs_before_step":5000, "priority":1, "cpu_index":0}
        for i in mlen:
            print(" >> message lenght = " + str(i))
            args["min_msg_size"] = i
            args["max_msg_size"] = i
            for j in range(0, len(self.pubs)):
                args["res_filenames"] = ['test0/' + self.subs[j][self.subs[j].rfind('/')+1:self.subs[j].rfind(' ')] + str(i) + '.json']
                s = self.create_process(self.subs[j], args)
                args["cpu_index"] = 1
                p = self.create_process(self.pubs[j], args)
                args["cpu_index"] = 0
                if s.poll() is None:
                    s.wait()
                s.stdout.close()
                s.stdin.close()
                if p.poll() is None:
                    p.wait()
                p.stdout.close()
                p.stdin.close()

    def test2(self):
        print(">>> running test2")
        try:
            os.mkdir("test2")
        except OSError:
            None
        args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":1000, "min_msg_size":65536, "max_msg_size":65536,  "step":0, "msgs_before_step":1000, "priority":1, "cpu_index":0, "interval":50}
        sub_count = range(1, 21)
        for i in sub_count:
            print(" >> number of subscribers - " + str(i))
            try:
                os.mkdir("test2/" + str(i))
            except OSError:
                None
            for k in range(0, len(self.pubs)):
                subs = []
                for j in range(0, i):
                    time.sleep(0.1)
                    args["res_filenames"] = ['test2/' + str(i) + '/' + self.subs[k][self.subs[k].rfind('/')+1:self.subs[k].rfind(' ')] + str(j) + '.json']
                    subs.append(self.create_process(self.subs[k], args))
                args["cpu_index"] = 1
                p = self.create_process(self.pubs[k], args)
                args["cpu_index"] = 0
                for s in subs:
                    if s.poll() is None:
                        s.wait()
                    s.stdout.close()
                    s.stdin.close()
                if p.poll() is None:
                    p.wait()
                p.stdout.close()
                p.stdin.close()

    def test4(self):
        print(">>> running test4")
        try:
            os.mkdir("test4")
        except OSError:
            None
        args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":204900, "min_msg_size":-896, "max_msg_size":2097152,  "step":1024, "msgs_before_step":100, "priority":1, "cpu_index":0, "interval":50}
        freq = [1, 5, 10, 15, 20, 25, 30]
        for f in freq:
            print(" >> freq = " + str(f))
            args["interval"] = 1000/f
            for j in range(0, len(self.pubs)):
                args["res_filenames"] = ['test4/' + self.subs[j][self.subs[j].rfind('/')+1:self.subs[j].rfind(' ')] + str(f) + '.json']
                s = self.create_process(self.subs[j], args)
                args["cpu_index"] = 1
                p = self.create_process(self.pubs[j], args)
                args["cpu_index"] = 0
                if s.poll() is None:
                    s.wait()
                s.stdout.close()
                s.stdin.close()
                if p.poll() is None:
                    p.wait()
                p.stdout.close()
                p.stdin.close()
            
    def test6(self):
        print(">>> running test6")
        try:
            os.mkdir("test6")
        except OSError:
            None
        args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":204900, "min_msg_size":-896, "max_msg_size":2097152,  "step":1024, "msgs_before_step":100, "priority":1, "cpu_index":0, "interval":0}
        for j in range(0, len(self.pubs)):
            args["res_filenames"] = ['test6/' + self.subs[j][self.subs[j].rfind('/')+1:self.subs[j].rfind(' ')] + str(i) + '.json']
            s = self.create_process(self.subs[j], args) 
            args["cpu_index"] = 1
            p = self.create_process(self.pubs[j], args)
            args["cpu_index"] = 0
            if s.poll() is None:
                s.wait()
            s.stdout.close()
            s.stdin.close()
            if p.poll() is None:
                p.wait()
            p.stdout.close()
            p.stdin.close()

if __name__ == "__main__":
    unittest.main()
