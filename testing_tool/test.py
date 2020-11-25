import unittest
import os
import subprocess
import json
from datetime import datetime
from general_funcs import log_file, get_configs, mk_nodedir, create_process, wait_and_end_process
from plotting import get_resfiles, get_grouped_filenames, plot_results, round_trip_grouped
from get_sys_info import system

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../Endurox/src/build/Endurox "]
    subs = ["../Endurox/src/build/Endurox "]
    nodes = []
    
    stype = 'subscriber'
    ptype = 'publisher'

    sys = system()

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
            self.sys.start(self.nodes[i])
            cwd = mk_nodedir(test_dir, self.nodes[i])
            for subtest_n, subtest in enumerate(configs):
                if len(configs) != 1:
                    print(datetime.now(), f" >>> subtest - {subtest_n+1}/{len(configs)}", file=log_file)
                subs = []
                pubs = []
                try:
                    if subtest[0].find('/') != -1:
                        os.mkdir(cwd + '/' + subtest[0][:subtest[0].find('/')])
                except OSError:
                    None
                for config in subtest:
                    print(datetime.now(), f"  >>> using config - {config}", file=log_file)
                    subs.append(create_process(prefix + self.subs[i], '../../../config/' + config, self.stype, cwd))
                if self.pairs:
                    for config in subtest:
                        pubs.append(create_process(prefix + self.pubs[i], '../../../config/' + config, self.ptype, cwd, True))
                else:
                    p = create_process(prefix + self.pubs[i], '../../../config/' + subtest[0], self.ptype, cwd, True)
                for sub_n, s in enumerate(subs):
                    wait_and_end_process(s)
                    print(datetime.now(), f"subscriber №{sub_n+1} finished", file=log_file)
                if self.pairs:
                    for pub_n, p in enumerate(pubs):
                        wait_and_end_process(p)
                        print(datetime.now(), f"publisher №{pub_n+1} finished", file=log_file)
                else:
                    wait_and_end_process(p)
                print(datetime.now(), "publisher finished", file=log_file, flush=True)
            self.sys.end(self.test_n)

    def test1(self):
        print(datetime.now(), ">>> running test1", file=log_file)
        self.test_n = 1
        self.subtests = False
        self.pairs = False
        self.startTest()

    def test2(self):
        print(datetime.now(), ">>> running test2", file=log_file)
        self.test_n = 2
        self.subtests = True
        self.pairs = False
        self.startTest()

    def test3(self):
        print(datetime.now(), ">>> running test3", file=log_file)
        self.test_n = 3
        self.subtests = False
        self.pairs = False
        self.startTest()

    def test4(self):
        print(datetime.now(), ">>> running test4", file=log_file)
        self.test_n = 4
        self.subtests = False
        self.pairs = False
        self.startTest()

    def test5(self):
        print(datetime.now(), ">>> running test5", file=log_file)
        self.test_n = 5
        self.subtests = False
        self.pairs = False
        self.startTest()
    
    def test6(self):
        print(datetime.now(), ">>> running test6", file=log_file)
        self.test_n = 6
        self.subtests = False
        self.pairs = False
        self.ptype = 'ping_pong'
        self.stype = 'ping_pong'
        self.startTest()
    
    def test7(self):
        print(datetime.now(), ">>> running test7", file=log_file)
        self.test_n = 7
        self.subtests = True
        self.pairs = True
        self.ptype = 'ping_pong'
        self.stype = 'ping_pong'
        self.startTest()

    def test8(self):
        print(datetime.now(), ">>> running test8", file=log_file)
        self.test_n = 8
        self.subtests = False
        self.pairs = False
        self.ptype = 'ping_pong'
        self.stype = 'ping_pong'
        self.startTest()

    def tearDown(self):
        resfiles = get_resfiles(self.test_n, self.subtests)
        self.sys.packet_loss(resfiles, self.test_n, self.subtests, self.test_n > 5)
        with open('system_info.json','w') as out:
            out.write(self.sys.get_info())
        if self.test_n == 2:
            for filenames in resfiles:
                plot_results([filenames], self.subtests)
        elif self.test_n < 6:
            resfiles = get_grouped_filenames(resfiles)
            for files in resfiles:
                plot_results(files)
        else:
            if self.test_n == 6:
                round_trip_grouped(resfiles)
            for filenames in resfiles:
                plot_results([filenames], self.test_n == 7, self.test_n > 5, 
                             grouping=(self.test_n<7))

if __name__ == "__main__":
    unittest.main()
    log_file.close()
