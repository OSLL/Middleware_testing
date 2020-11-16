## NATS

### Dependencies

* [Golang version >=1.15](https://github.com/golang/go/wiki/Ubuntu)
* NATS go client library:
```
go get -u github.com/nats-io/nats.go
```
* [NATS server](https://docs.nats.io/nats-server/installation#downloading-a-release-build). After unzipping, add bin directory to PATH like this:
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
go build -o NATS main.go
```

### Running

To run client:
```
./go/src/main/NATS -c <config> -t <node_type>
```
To run server user following command:
```
nats-server -c server-config.txt
```
