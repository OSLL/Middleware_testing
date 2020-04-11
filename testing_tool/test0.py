import os
from general_funcs import create_process, wait_and_end_process

def test0(pubs, subs):
    pub_filenames = []
    sub_filenames = []
    #filenames = ['pub', 'sub']
    try:
        os.mkdir("test0")
    except OSError:
        None
    mlen = [50, 60000]
    args = {"topic":'test_topic', "res_filenames":['pub', 'sub'], "m_count":5000, "min_msg_size":100, "max_msg_size":100, "step":0, "msgs_before_step":5000, "priority":[99, 99], "cpu_index":[0, 1], "topic_priority":100, "interval":0}
    for i in mlen:
        print(" >> message lenght = " + str(i))
        args["min_msg_size"] = i
        args["max_msg_size"] = i
        for j in range(0, len(pubs)):
            args["res_filenames"][0] = 'test0/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + str(i) + '_pub.json'
            args["res_filenames"][1] = 'test0/' + subs[j][subs[j].rfind('/')+1:subs[j].rfind(' ')] + str(i) + '_sub.json'
            pub_filenames.append(args["res_filenames"][0])
            sub_filenames.append(args["res_filenames"][1])
            s = create_process(subs[j], args)
            p = create_process(pubs[j], args)
            wait_and_end_process(s)
            wait_and_end_process(p)
    return (pub_filenames, sub_filenames)
