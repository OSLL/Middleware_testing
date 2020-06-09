import os
from general_funcs import mkdir_config, constr_resfilename, constr_config


def test7_config():
    configs = []
    mkdir_config(7)
    args = {"topic":['test_topic', 'test_topic1'], "res_filenames":['pub', 'sub'], 
            "m_count":800, "min_msg_size":-896, "max_msg_size":2*1024*1024, 
            "step":256*1024, "msgs_before_step":100, "priority":[-1, -1],
            "cpu_index":[-1, -1], "interval":50, "topic_priority":100}
    args["res_filenames"][0] = constr_resfilename('', 'p')
    args["res_filenames"][1] = constr_resfilename('', 's')
    configs.append(constr_config(7, 'config', args))
    return configs
