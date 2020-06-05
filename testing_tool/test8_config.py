import os
from general_funcs import mkdir_config, constr_resfilename, constr_config


def test8_config():
    configs = []
    mkdir_config(8)
    args = {"topic":['test_topic', 'test_topic1'], 
            "res_filenames":['pub', 'sub'],
            "m_count":10000, "min_msg_size":64*1024, "max_msg_size":2097152,
            "step":64*1024, "msgs_before_step":100,
            "priority":[-1, -1], "cpu_index":[-1, -1], 
            "interval":0, "topic_priority":100}
    args["res_filenames"][0] = constr_resfilename('first', 'p')
    args["res_filenames"][1] = constr_resfilename('second', 's')
    configs.append(constr_config(8, 'config', args))
    return configs
