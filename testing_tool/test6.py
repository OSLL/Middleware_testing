import os
from general_funcs import create_process, wait_and_end_process

def test6(pubs, subs):
    try:
        os.mkdir("test6")
    except OSError:
        None
    args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":204900, "min_msg_size":-896, "max_msg_size":2097152,  "step":1024, "msgs_before_step":100, "priority":99, "cpu_index":0, "interval":0}
    for j in range(0, len(pubs)):
        args["res_filenames"] = ['test6/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + '.json']
        s = create_process(subs[j], args)
        args["cpu_index"] = 1
        p = create_process(pubs[j], args)
        args["cpu_index"] = 0
        wait_and_end_process(s)
        wait_and_end_process(p)

