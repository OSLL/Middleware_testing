import unittest
import os
import subprocess


class MiddlewareTesting(unittest.TestCase):
    nodes = ['../young/build/FastRTPSTest']
    pub = " subscriber " 
    sub = " publisher "

    def test0(self):
        mlen = [50, 60000]
        args = ['test_topic', 'json', '5000', '0', '100',  '0', '100', '1', '0']
        for i in mlen:
            args[4] = str(i)
            for node in self.nodes:
                args[1] = node[node.rfind('/')+1:] + str(i) + '.json'
                s = subprocess.Popen(node+self.sub+' '.join(args), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")
                args[8] = '1'
                p = subprocess.Popen(node+self.pub+' '.join(args), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")
                args[8] = '0'
                s.wait()
                p.wait()

if __name__ == "__main__":
    unittest.main()

