import os
from general_funcs import create_process, wait_and_end_process

def test4(pubs, subs):
    pub_filenames = []
    sub_filenames = []
    try:
        os.mkdir("test4")
    except OSError:
        None
    args = {"topic":'test_topic', "res_filenames":['pub', 'sub'], "m_count":204900, "min_msg_size":-896, "max_msg_size":2097152,  "step":1024, "msgs_before_step":100, "priority":[99, 99], "cpu_index":[0, 1], "interval":50, "topic_priority":100}
    freq = [1, 5, 10, 15, 20, 25, 30]
    for f in freq:
        print(" >> freq = " + str(f))
        args["interval"] = 1000/f
        for j in range(0, len(pubs)):
            args["res_filenames"][0] = 'test4/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + str(f) + '_pub.json'
            args["res_filenames"][1] = 'test4/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + str(f) + '_sub.json'
            pub_filenames.append(args["res_filenames"][0])
            sub_filenames.append(args["res_filenames"][1])
            s = create_process(subs[j], args)
            p = create_process(pubs[j], args)
            wait_and_end_process(s)
            wait_and_end_process(p)
    return (pub_filenames, sub_filenames)
