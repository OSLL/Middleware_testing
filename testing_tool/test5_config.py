import os
from general_funcs import mkdir_config, constr_resfilename, constr_config

def test5_config():
    configs = []
    mkdir_config(5)
    args = {"topic":'test_topic', "res_filenames":['pub', 'sub'], "m_count":3300, "min_msg_size":-896, "max_msg_size":2097152,  "step":64*1024, "msgs_before_step":100, "priority":[99, 99], "cpu_index":[0, 1], "interval":50, "topic_priority":100}
    freq = [1, 5, 10, 15, 20, 25, 30]
    for f in freq:
        args["interval"] = 1000/f
        args["res_filenames"][0] = constr_resfilename(f, 'p')
        args["res_filenames"][1] = constr_resfilename(f, 's')
        configs.append(constr_config(5, f, args))
    return configs
