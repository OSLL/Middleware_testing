import os
from general_funcs import mkdir_config, constr_resfilename, constr_config

def test3_config():
    configs = []
    mkdir_config(3)
    args = {"topic":'test_topic', "res_filenames":['pub', 'sub'], "m_count":1000, "min_msg_size":65536, "max_msg_size":65536,  "step":0, "msgs_before_step":1000, "priority":[99, 99], "cpu_index":[0, 1], "interval":50, "topic_priority":100}
    sub_count = range(1, 21)
    for i in sub_count:
        try:
            os.mkdir('test_3/config/' + str(i))
        except OSError:
            None
        args["res_filenames"][0] = constr_resfilename(str(i)+'/', 'p')
        for j in range(0, i):
            args["res_filenames"][1] = constr_resfilename(str(i)+'/'+str(j), 's')
            configs.append(constr_config(3, str(i)+'/'+str(j), args))
    return configs
