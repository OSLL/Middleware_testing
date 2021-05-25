## NSQ

### Dependencies

* [Golang version >=1.15](https://github.com/golang/go/wiki/Ubuntu)
* NSQ go client library:
```
go get -u github.com/nsqio/go-nsq
```
* [NSQ server](https://nsq.io/deployment/installing.html). After unzipping, add bin directory to PATH like this:
```
export PATH=$PATH:$HOME/unzip_path/bin
```

### Building

Add working directory to GOPATH:
```
export GOPATH=$GOPATH:`pwd`/go
```
Then build:
```
cd go/src/main
go build -o NSQ main.go
```

### Running

To run client:
```
./go/src/main/NSQ -c <config> -t <node_type>
```
To run server use start_nsq.sh script or run following command:
```
nsqlookupd &
nsqd --lookupd-tcp-address=127.0.0.1:4160 --max-msg-size=2200000 &
nsqadmin --lookupd-http-address=127.0.0.1:4161 &
```
