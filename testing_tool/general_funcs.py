import subprocess
import json

def create_process(name, args):
    with open('args.json', 'w') as f:
        json.dump(args, f)
    return subprocess.Popen(name+' args.json', shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=".")

def wait_and_end_process(process):
    end = str.encode("end")
    process.communicate(end)
    if process.poll() is None:
        process.wait()
    if process.poll() != 0:
        print("Process finished incorrectly, exit code", process.poll())
    process.stdout.close()
    process.stdin.close()
