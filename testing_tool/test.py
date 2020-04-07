import unittest
import os
import subprocess
import json
import time
from test0 import test0
from test2 import test2
from test4 import test4
from test6 import test6

class MiddlewareTesting(unittest.TestCase):
    pubs = ["../FastRTPS/test/build/FastRTPSTest publisher"]
    subs = ["../FastRTPS/test/build/FastRTPSTest subscriber"]

    def test0(self):
        print(">>> running test0")
        test0(self.pubs, self.subs)

    def test2(self):
        print(">>> running test2")
        test2(self.pubs, self.subs)

    def test4(self):
        print(">>> running test4")
        test4(self.pubs, self.subs)
            
    def test6(self):
        print(">>> running test6")
        test6(self.pubs, self.subs)

if __name__ == "__main__":
    unittest.main()
