import subprocess
import json
import os
from datetime import datetime
from general_funcs import log_file, wait_and_end_process, get_all_pids
from tracer import CopyingTracer

def get_msg_ledger():
    return {"name": "message_ledger", "type": "isaac::alice::MessageLedger"}

def get_control_node():
    return {"name": "control", "components": [get_msg_ledger(), {"name": "control", "type": "Control"}]}

def get_pub_node(number):
    return {"name": f"z_pub{number}", "components": [get_msg_ledger(), {"name": "pub", "type": "Publisher"}]}

def get_sub_node(number):
    return {"name": f"sub{number}", "components": [get_msg_ledger(), {"name": "sub", "type": "Subscriber"}]}

def get_ping_pong_node(number):
    return {"name": f"ping_pong{number}", "components": [get_msg_ledger(), {"name": "ping_pong", "type": "PingPong"}]}

def get_pub_sub_node(number):
    return {"name": f"ping_pong{number}", "components": [get_msg_ledger(), {"name": "pub", "type": "Publisher"}, {"name": "sub", "type": "Subscriber"}]}

def get_config(config_filename, isPingPong, out_dir, isPubOrFirst = False):
    with open(config_filename, 'r') as f:
        data = json.load(f)
    if isPubOrFirst:
        index = 0
    else:
        index = 1
    config = {}
    config['tick_period'] = f'{data["interval"]}ms'
    config['msg_count'] = data['m_count']
    config['prior'] = data['priority'][index]
    config['cpu_index'] = data['cpu_index'][index]
    config['min_msg_size'] = data['min_msg_size']
    config['max_msg_size'] = data['max_msg_size']
    config['step'] = data['step']
    config['msgs_before_step'] = data['msgs_before_step']
    config['filename'] = out_dir + data['res_filenames'][index]
    config['ping_pong'] = isPingPong
    if isPingPong:
        if data['interval'] != 0 and isPubOrFirst:
            return {"sub": config, "pub": config}
        else:
            config['first'] = isPubOrFirst
            return {"ping_pong": config} 
    else:
        if isPubOrFirst:
            return {"pub": config}
        else:
            return {"sub": config}


    

def isaac_generate_config(test_n, configs, isPingPong = False,  filename = "testing.app.json"):
    prefix_dir = f'test_{test_n}/config/'
    res_dir = os.getcwd() + f'/test_{test_n}/results/Isaac/data/'
    config = {"name": "testing" , "modules": ["testing:nodes"], "graph": {"nodes": [], "edges": []}, "config":{}}
    config['graph']['nodes'].append(get_control_node())
    if isPingPong:
        config['config']['control'] = {'control': {'node_count': len(configs) * 2}}
        for i, config_name in enumerate(configs):
            config['config'][f'ping_pong{2 * i}'] = get_config(prefix_dir + config_name, isPingPong, res_dir)
            config['config'][f'ping_pong{2 * i + 1}'] = get_config(prefix_dir + config_name, isPingPong, res_dir, True)
            config['graph']['nodes'].append(get_ping_pong_node(2 * i))
            if config['config'][f'ping_pong{2 * i}']['ping_pong']['tick_period'] != '0ms':
                config['graph']['nodes'].append(get_pub_sub_node(2 * i + 1))
                config['graph']['edges'].extend([{'source': f'ping_pong{2 * i + 1}/pub/send', 'target': f'ping_pong{2 * i}/ping_pong/receive'},
                                                {'source': f'ping_pong{2 * i}/ping_pong/send', 'target': f'ping_pong{2 * i + 1}/sub/receive'}])
            else:
                config['graph']['nodes'].append(get_ping_pong_node(2 * i + 1))
                config['graph']['edges'].extend([{'source': f'ping_pong{2 * i + 1}/ping_pong/send', 'target': f'ping_pong{2 * i}/ping_pong/receive'},
                                                {'source': f'ping_pong{2 * i}/ping_pong/send', 'target': f'ping_pong{2 * i + 1}/ping_pong/receive'}])
    else:
        config['config']['control'] = {'control': {'node_count': len(configs) + 1}}
        for i, config_name in enumerate(configs):
            config['graph']['nodes'].append(get_sub_node(i))
            config['graph']['edges'].append({"source": 'z_pub0/pub/send', 'target': f"sub{i}/sub/receive"})
            config['config'][f'sub{i}'] = get_config(prefix_dir + config_name, isPingPong, res_dir)
        config['graph']['nodes'].append(get_pub_node(0))
        config['config']['z_pub0'] = get_config(prefix_dir + configs[0], isPingPong, res_dir, True)
    with open(filename, 'w') as f:
        f.write(json.dumps(config, indent=2, separators=(',', ': ')))


def run_isaac(isaac_dir, filename, withTracer = False, tracer_out_dir = ''):
    #isaac_dir -- absolutely path to folder where exist dir sdk
    #filename -- absolutely path to .app.json
    if withTracer:
        tracer = CopyingTracer()
    process = subprocess.Popen(f"./packages/testing/main --app {filename}", 
        shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
        close_fds=True, cwd = f"{isaac_dir}/sdk/bazel-bin")
    if withTracer:
        _, isaac_pids = get_all_pids(process, process)
    wait_and_end_process(process)
    if withTracer:
        tracer.close()
        tracer.write_results((None,isaac_pids), (None, []), tracer_out_dir)
    print('Isaac Return code:', process.returncode)

