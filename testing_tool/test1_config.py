from general_funcs import mkdir_config, constr_resfilename, constr_config


def test1_config():
    configs = []
    mkdir_config(1)
    mlen = [50, 60000]
    args = {"topic":['test_topic', 'test_topic1'], "res_filenames":['pub', 'sub'], 
            "m_count":5000, "min_msg_size":100, "max_msg_size":100, 
            "step":0, "msgs_before_step":5000, "priority":[99, 99], 
            "cpu_index":[0, 1], "topic_priority":100, "interval":0}
    for i in mlen:
        args["min_msg_size"] = i
        args["max_msg_size"] = i
        args["res_filenames"][0] = constr_resfilename(i, 'p')
        args["res_filenames"][1] = constr_resfilename(i, 's')
        configs.append(constr_config(1, i, args))
    return configs
