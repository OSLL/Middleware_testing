import os
import subprocess
import json
from os.path import isfile, isdir, join


def get_configs(test_n, subtests=False):
    directory = f'test_{test_n}/config'
    if subtests:
        dirs = [d for d in os.listdir(directory) if isdir(join(directory, d))]
        return [[join(d, f) for f in os.listdir(join(directory, d)) 
                 if isfile(join(directory, d, f)) and f.endswith('.json')] 
                 for d in dirs]
    return [f for f in os.listdir(directory) 
            if isfile(join(directory, f)) and f.endswith('.json')]


def get_resfiles(test_n, subtest=False):
    directory = f'test_{test_n}/results'
    dirs = [join(directory, d) + '/data' for d in os.listdir(directory) 
            if isdir(join(directory, d))]
    if subtest:
        ldirs = [[join(direct, d) for d in os.listdir(direct) 
                  if isdir(join(direct, d))] for direct in dirs]
        res = []
        for dirs in ldirs:
            subres = []
            for d in dirs:
                subres.append([join(d, f) for f in os.listdir(d) 
                               if isfile(join(d, f))])
            res.append(subres)
        return res
    res = []
    for direct in dirs:
        res += [join(direct, f) for f in os.listdir(direct) 
                if isfile(join(direct, f)) and f.endswith('.json')]
    return res


def create_process(name, config, cwd):
    command = f'{name} "{config}"'
    return subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=cwd)


def wait_and_end_process(process):
    end = str.encode("end")
    process.communicate(end)
    if process.poll() is None:
        process.wait()
    if process.poll() != 0:
        print("Process finished incorrectly, exit code", process.poll())
    process.stdout.close()
    process.stdin.close()


def mk_nodedir(test_dir, node):
    cwd = f'{test_dir}/results/node'
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
