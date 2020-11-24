import os
from general_funcs import mkdir_config, constr_resfilename, constr_config


def test3_config():
    configs = []
    mkdir_config(3)
    args = {"topic":['test_topic', 'test_topic1'], 
            "res_filenames":['pub', 'sub'], 
            "m_count":800, "min_msg_size":128-256*1024, "max_msg_size":2*1024*1024,
            "step":256*1024, "msgs_before_step":100, "priority":[99, 99], 
            "cpu_index":[-1, -1], "interval":50, "topic_priority":100}
    freq = [1, 5, 10, 15, 20, 25, 30, 35, 40, 50, 100, 200, 400, 600, 800, 1000]
    for f in freq:
        args["interval"] = 1000/f
        args["res_filenames"][0] = constr_resfilename(f, 'p')
        args["res_filenames"][1] = constr_resfilename(f, 's')
        configs.append(constr_config(3, f, args))
    return configs

