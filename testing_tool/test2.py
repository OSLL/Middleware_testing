import os
import time
from general_funcs import create_process, wait_and_end_process

def test2(pubs, subs):
    pub_filenames = []
    sub_filenames = []
    try:
        os.mkdir("test2")
    except OSError:
        None
    args = {"topic":'test_topic', "res_filenames":['pub', 'sub'], "m_count":1000, "min_msg_size":65536, "max_msg_size":65536,  "step":0, "msgs_before_step":1000, "priority":[99, 99], "cpu_index":[0, 1], "interval":50, "topic_priority":100}
    sub_count = range(1, 3)
    for i in sub_count:
        print(" >> number of subscribers - " + str(i))
        try:
            os.mkdir("test2/" + str(i))
        except OSError:
            None
        for k in range(0, len(pubs)):
            snodes = []
            for j in range(0, i):
                time.sleep(0.1)
                args["res_filenames"][1] = 'test2/' + str(i) + '/' + subs[k][subs[k].rfind('/')+1:subs[k].rfind(' ')] + str(j) + '_sub.json'
                sub_filenames.append(args["res_filenames"][1])
                snodes.append(create_process(subs[k], args))
            args["res_filenames"][0] = 'test2/' + str(i) + '/' + subs[k][subs[k].rfind('/')+1:subs[k].rfind(' ')] + str(j) + '_pub.json'
            pub_filenames.append(args["res_filenames"][0])
            p = create_process(pubs[k], args)
            for s in snodes:
                wait_and_end_process(s)
            wait_and_end_process(p)
    return (pub_filenames, sub_filenames)
