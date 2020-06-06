import unittest
import os
import subprocess
import json
from datetime import datetime
from general_funcs import log_file, get_configs, mk_nodedir, create_process, wait_and_end_process
from plotting import get_resfiles, plot_results

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../FastRTPS/test/build/FastRTPSTest"]
    subs = ["../FastRTPS/test/build/FastRTPSTest"]
    nodes = []
    
    stype = 'subscriber'
    ptype = 'publisher'

    @classmethod
    def setUpClass(self):
        for p in self.pubs:
            start = p.find('/')
            end = p[start+1:].find('/')
            self.nodes.append(p[start+1 : start+end+1])

    def startTest(self):
        test_dir = 'test_' + str(self.test_n)
        configs = get_configs(self.test_n, self.subtests)
        if isinstance(configs[0], str):
            configs = [[config] for config in configs]
        try:
            os.mkdir(test_dir + '/results/')
        except OSError:
            None
        prefix = '../../../../'
        for i in range(0, len(self.pubs)):
            print(datetime.now(), " >>> testing " + self.nodes[i], file=log_file)
            cwd = mk_nodedir(test_dir, self.nodes[i])
            for subtest_n, subtest in enumerate(configs):
                if len(configs) != 1:
                    print(datetime.now(), f" >>> subtest - {subtest_n+1}/{len(configs)}", file=log_file)
                subs = []
                try:
                    if subtest[0].find('/') != -1:
                        os.mkdir(cwd + '/' + subtest[0][:subtest[0].find('/')])
                except OSError:
                    None
                for config in subtest:
                    print(datetime.now(), f"  >>> using config - {config}", file=log_file)
                    subs.append(create_process(prefix + self.subs[i], '../../../config/' + config, self.stype, cwd))
                p = create_process(prefix + self.pubs[i], '../../../config/' + subtest[0], self.ptype, cwd, True)
                for sub_n, s in enumerate(subs):
                    wait_and_end_process(s)
                    print(datetime.now(), f"subscriber №{sub_n+1} finished", file=log_file)
                wait_and_end_process(p)
                print(datetime.now(), "publisher finished", file=log_file, flush=True)

    def test0(self):
        print(datetime.now(), ">>> running test0", file=log_file)
        self.test_n = 0
        self.subtests = False
        self.startTest()

    def test3(self):
        print(datetime.now(), ">>> running test3", file=log_file)
        self.test_n = 3
        self.subtests = True
        self.startTest()

    def test5(self):
        print(datetime.now(), ">>> running test5", file=log_file)
        self.test_n = 5
        self.subtests = False
        self.startTest()

    def test6(self):
        print(datetime.now(), ">>> running test6", file=log_file)
        self.test_n = 6
        self.subtests = False
        self.startTest()

    def test7(self):
        print(datetime.now(), ">>> running test7", file=log_file)
        self.test_n = 7
        self.subtests = False
        self.startTest()
    
    def test8(self):
        print(datetime.now(), ">>> running test8", file=log_file)
        self.test_n = 8
        self.subtests = False
        self.ptype = 'ping_pong'
        self.stype = 'ping_pong'
        self.startTest()

    def tearDown(self):
        resfiles = get_resfiles(self.test_n, self.subtests)
        for filename in resfiles:
            plot_results([filename], self.test_n == 8)


if __name__ == "__main__":
    unittest.main()
    log_file.close()
