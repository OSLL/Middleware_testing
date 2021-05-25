import os
from general_funcs import mkdir_config, constr_resfilename, constr_config


def test4_config():
    configs = []
    mkdir_config(4)
    args = {"topic":['test_topic', 'test_topic1'], "res_filenames":['pub', 'sub'], 
            "m_count":5000, "min_msg_size":64*1024, "max_msg_size":64*1024,
            "step":0, "msgs_before_step":100, "priority":[99, 99], 
            "cpu_index":[0, 1], "interval":0, "topic_priority":100}
    cpu_index = [[-1, -1], [0, 1], [-1, 1], [0, -1]]
    for i in cpu_index:
        args["cpu_index"] = i
        args["res_filenames"][0] = constr_resfilename(i, 'p')
        args["res_filenames"][1] = constr_resfilename(i, 's')
        configs.append(constr_config(4, i, args))
    return configs

