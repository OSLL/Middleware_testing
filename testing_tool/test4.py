import os
from general_funcs import create_process, wait_and_end_process

def test4(pubs, subs):
    res_filenames = []
    try:
        os.mkdir("test4")
    except OSError:
        None
    args = {"topics":['test_topic'], "res_filenames":['json'], "m_count":204900, "min_msg_size":-896, "max_msg_size":2097152,  "step":1024, "msgs_before_step":100, "priority":99, "cpu_index":0, "interval":50}
    freq = [1, 5, 10, 15, 20, 25, 30]
    for f in freq:
        print(" >> freq = " + str(f))
        args["interval"] = 1000/f
        for j in range(0, len(pubs)):
            args["res_filenames"] = ['test4/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + str(f) + '.json']
            res_filenames.append(args["res_filenames"][0])
            s = create_process(subs[j], args)
            args["cpu_index"] = 1
            p = create_process(pubs[j], args)
            args["cpu_index"] = 0
            wait_and_end_process(s)
            wait_and_end_process(p)
    return res_filenames
