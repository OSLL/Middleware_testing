import os
from general_funcs import create_process, wait_and_end_process

def test0(pubs, subs):
    res_filenames = []
    try:
        os.mkdir("test0")
    except OSError:
        None
    mlen = [50, 60000]
    args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":5000, "min_msg_size":100, "max_msg_size":100,  "step":0, "msgs_before_step":5000, "priority":99, "cpu_index":0}
    for i in mlen:
        print(" >> message lenght = " + str(i))
        args["min_msg_size"] = i
        args["max_msg_size"] = i
        for j in range(0, len(pubs)):
            args["res_filenames"] = ['test0/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + str(i) + '.json']
            res_filenames.append(args["res_filenames"][0])
            s = create_process(subs[j], args)
            args["cpu_index"] = 1
            p = create_process(pubs[j], args)
            args["cpu_index"] = 0
            wait_and_end_process(s)
            wait_and_end_process(p)
    return res_filenames
