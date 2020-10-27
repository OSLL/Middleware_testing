# OpenSplice testing guid

## OpenSplice building

1. clone repo or download source files from: https://github.com/ADLINK-IST/opensplice
2. Install dependences:
    * gcc - 3.2.x or above
    * gmake - 3.80 or above
    * gawk - Any version is acceptable
    * flex - Any version is acceptable
    * bison - Version 2.7 or above is required.
    * perl - Version 5.8 or above is required.
3. Build OpenSplice using following commands:
    * ```./configure``` 
    * ```make```
    * ```make install```
4. Before building project or running the program it's necessary to config environment:
    ```source /opensplice/install/HDE/x86_64.linux/release.com```
5. Copy opensplice config from OpenSplice/ospl.xml to $OSPL_HOME/install/HDE/x86_64.linux/etc/ospl.xml
    ```cp ospl.xml $OSPL_HOME/install/HDE/x86_64.linux/etc/ospl.xml
 
## Building project

1. ```cd dds_testing/OpenSplice/src/```
2. ```cmake .```
3. ```make```

## Running program

To run the program use command: ```./opensplice_node```
To get info about parameters use: ```./opensplice_node --help```

