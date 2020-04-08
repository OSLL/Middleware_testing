import unittest
import os
import subprocess
import json
import time
from test0 import test0
from test2 import test2
from test4 import test4
from test6 import test6
from plotting import plot_results

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../FastRTPS/test/build/FastRTPSTest publisher"]
    subs = ["../FastRTPS/test/build/FastRTPSTest subscriber"]

    def test0(self):
        print(">>> running test0")
        self.res_filenames = test0(self.pubs, self.subs)

    @unittest.skip("")
    def test2(self):
        print(">>> running test2")
        self.res_filenames = test2(self.pubs, self.subs)

    @unittest.skip("")
    def test4(self):
        print(">>> running test4")
        self.res_filenames = test4(self.pubs, self.subs)
            
    @unittest.skip("")
    def test6(self):
        print(">>> running test6")
        self.res_filenames = test6(self.pubs, self.subs)

    def tearDown(self):
        for filename in self.res_filenames:
            plot_results(filename)

if __name__ == "__main__":
    unittest.main()
