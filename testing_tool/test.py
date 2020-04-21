import unittest
import os
import subprocess
import json
import time
from general_funcs import get_configs, get_resfiles, mk_nodedir, create_process, wait_and_end_process
from plotting import plot_results

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../FastRTPS/test/build/FastRTPSTest publisher"]
    subs = ["../FastRTPS/test/build/FastRTPSTest subscriber"]
    nodes = []

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
            print(" >>> testing " + self.nodes[i])
            cwd = mk_nodedir(test_dir, self.nodes[i])
            for subtest_n, subtest in enumerate(configs):
                if len(configs) != 1:
                    print(f" >>> subtest - {subtest_n+1}/{len(configs)}")
                subs = []
                try:
                    if subtest[0].find('/') != -1:
                        os.mkdir(cwd + '/' + subtest[0][:subtest[0].find('/')])
                except OSError:
                    None
                for config in subtest:
                    print(f"  >>> using config - {config}")
                    subs.append(create_process(prefix + self.subs[i], '../../../config/' + config, cwd))
                p = create_process(prefix + self.pubs[i], '../../../config/' + subtest[0], cwd)
                for sub_n, s in enumerate(subs):
                    wait_and_end_process(s)
                    print(f"subscriber №{sub_n+1} finished")
                wait_and_end_process(p)
                print("publisher finished")

    def test0(self):
        print(">>> running test0")
        self.test_n = 0
        self.subtests = False
        self.startTest()

    def test3(self):
        print(">>> running test3")
        self.test_n = 3
        self.subtests = True
        self.startTest()

    def test5(self):
        print(">>> running test5")
        self.test_n = 5
        self.subtests = False
        self.startTest()

    def test6(self):
        print(">>> running test6")
        self.test_n = 6
        self.subtests = False
        self.startTest()

    def test7(self):
        print(">>> running test7")
        self.test_n = 7
        self.subtests = False
        self.startTest()

    def tearDown(self):
        resfiles = get_resfiles(self.test_n, self.subtests)
        for filename in resfiles:
            plot_results([filename])


if __name__ == "__main__":
    unittest.main()
