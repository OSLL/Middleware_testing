# OpenSplice testing guid

## Open62541 installing

1. Adding repo: ```sudo add-apt-repository ppa:open62541-team/ppa```
2. Updating repo info: ```sudo apt-get update```
3. Installing: ```sudo apt-get install libopen62541-1-dev```
    
## Building project

1. ```cd dds_testing/open62541/src/```
2. ```mkdir build && cd build```
3. ```cmake ..```
4. ```make```

## Running program

To run the program use command: ```./node```
To get info about parameters use: ```./node --help```