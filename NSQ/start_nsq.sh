nsqlookupd & 
nsqd --lookupd-tcp-address=127.0.0.1:4160 --max-msg-size=2200000 &
nsqadmin --lookupd-http-address=127.0.0.1:4161 &
