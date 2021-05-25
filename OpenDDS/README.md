## Requirements

gcc >= 9

g++ >= 9

https://github.com/hbristow/argparse

https://github.com/nlohmann/json

## How to build OpenDDS

git clone https://github.com/objectcomputing/OpenDDS.git

./configure --prefix=/usr

make -j

make -j install

## How to run

source <OpenDDS_install_path>/setenv.sh

DCPSInfoRepo -o repo.ior to start DDS domain

publisher/subscriber -DCPSConfigFile shmem.ini
