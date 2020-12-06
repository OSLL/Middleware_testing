# CycloneDDS testing guide

## CycloneDDS building

1. clone repo or download source files from: https://github.com/eclipse-cyclonedds/cyclonedds/tree/releases/0.7.x
2. Install dependences:
    * gcc 
    * OpenSSL
    * Java JDK, version 8 or later
    * Apache Maven, version 3.5 or later

Command for ubuntu:

```apt install maven default-jdk```

3. Build CycloneDDS using following commands:
    * ```cd cyclonedds```
    * ```git checkout releases/0.7.x```
    * ```mkdir build && cd build``` 
    * ```cmake ..```
    * ```make -j4```
    * ```make install```
 
## Building project

1. Set correct path to the CycloneDDS project (Set parameter CYCLONEDDS_DIR in CMakeLists.txt )
2. ```cd dds_testing/CycloneDDS```
3. ```cmake .```
4. ```make```

## Running program

To run the program use command: ```./cyclonedds_node```
To get info about parameters use: ```./cyclonedds_node --help```

