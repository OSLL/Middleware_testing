import os
from general_funcs import mkdir_config, constr_resfilename, constr_config


def test7_config():
    configs = []
    mkdir_config(7)
    args = {"topic":'test_topic', "res_filenames":['pub', 'sub'], 
            "m_count":3300, "min_msg_size":-896, "max_msg_size":2097152, 
            "step":64*1024, "msgs_before_step":100, "priority":[-1, -1],
            "cpu_index":[-1, -1], "interval":0, "topic_priority":100}
    args["res_filenames"][0] = constr_resfilename('', 'p')
    args["res_filenames"][1] = constr_resfilename('', 's')
    configs.append(constr_config(7, 'config', args))
    return configs
