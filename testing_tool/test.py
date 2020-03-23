import unittest
import os
import subprocess
import json
import time

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../without_middle_class/build/FastRTPSTest publisher"]
    subs = ["../without_middle_class/build/FastRTPSTest subscriber"]

    def test0(self):
        print(">>> running test0")
        try:
            os.mkdir("test0")
        except OSError:
            None
        mlen = [50, 60000]
        args = {"topic":'test_topic', "res_filename":'json', "m_count":5000, "min_msg_size":100, "max_msg_size":100,  "step":0, "msgs_before_step":5000, "priority":1, "cpu_index":0}
        for i in mlen:
            print(" >> message lenght = " + str(i))
            args["min_msg_size"] = i
            args["max_msg_size"] = i
            for j in range(0, len(self.pubs)):
                args["res_filename"] = 'test0/' + self.subs[j][self.subs[j].rfind('/')+1:self.subs[j].rfind(' ')] + str(i) + '.json'
                with open('args.json', 'w') as f:
                    json.dump(args, f)
                s = subprocess.Popen(self.subs[j]+' '+'args.json', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")
                args["cpu_index"] = 1
                with open('args.json', 'w') as f:
                    json.dump(args, f)
                p = subprocess.Popen(self.pubs[j]+' '+'args.json', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")
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
        args = {"topic":'test_topic', "res_filename":'json', "m_count":1000, "min_msg_size":50, "max_msg_size":50,  "step":0, "msgs_before_step":1000, "priority":1, "cpu_index":0}
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
                    args["res_filename"] = 'test2/' + str(i) + '/' + self.subs[k][self.subs[k].rfind('/')+1:self.subs[j].rfind(' ')] + str(j) + '.json'
                    with open('args.json', 'w') as f:
                        json.dump(args, f)
                    subs.append(subprocess.Popen(self.subs[k]+' '+'args.json', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd="."))
                args["cpu_index"] = 1
                with open('args.json', 'w') as f:
                    json.dump(args, f)
                p = subprocess.Popen(self.pubs[k]+' '+'args.json', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")
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


if __name__ == "__main__":
    unittest.main()
