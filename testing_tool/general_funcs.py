import os
import subprocess
import json
from datetime import datetime
from os.path import isfile, isdir, join

log_file = open('log.txt', 'w')

def get_configs(test_n, subtests=False):
    directory = f'test_{test_n}/config'
    if subtests:
        dirs = [d for d in os.listdir(directory) if isdir(join(directory, d))]
        return [[join(d, f) for f in os.listdir(join(directory, d)) 
                 if isfile(join(directory, d, f)) and f.endswith('.json')] 
                 for d in dirs]
    return [f for f in os.listdir(directory) 
            if isfile(join(directory, f)) and f.endswith('.json')]


def create_process(name, config, ntype, cwd, isFirst=False):
    command = f'{name} -c {config} -t {ntype}'
    print(command)
    if isFirst:
        command += ' --first'
    return subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=cwd)


def wait_and_end_process(process):
    end = str.encode("end")
    out, err = process.communicate(end)
    if err is not None:
        print(datetime.now(), err, file=log_file)
    if process.poll() is None:
        process.wait()
    if process.poll() != 0:
        print(datetime.now(), "Process finished incorrectly, exit code", process.poll(), file=log_file)
    process.stdout.close()
    process.stdin.close()


def mk_nodedir(test_dir, node):
    cwd = f'{test_dir}/results/{node}'
    try:
        os.mkdir(cwd)
    except OSError:
        None
    cwd += '/data'
    try:
        os.mkdir(cwd)
    except OSError:
        None
    return cwd


def mkdir_config(test_n):
    try:
        os.mkdir(f"test_{test_n}")
    except OSError:
        None
    try:
        os.mkdir(f"test_{test_n}/config")
    except OSError:
        None


def constr_config(test_n, param, args):
    name = f'test_{test_n}/config/{param}.json'
    with open(name, 'w') as f:
        json.dump(args, f)
    return name


def constr_resfilename(param, pub_or_sub):
    return f'{param}_{pub_or_sub}ub.json'
