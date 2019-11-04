#!/bin/bash
mkdir /sys/fs/cgroup/cpuset/sub_cpuset
/bin/echo 0 > /sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpus
/bin/echo 1 > /sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpu_exclusive
/bin/echo 0 > /sys/fs/cgroup/cpuset/sub_cpuset/cpuset.mems
mkdir /sys/fs/cgroup/cpuset/pub_cpuset
/bin/echo 1 > /sys/fs/cgroup/cpuset/pub_cpuset/cpuset.cpus
/bin/echo 1 > /sys/fs/cgroup/cpuset/pub_cpuset/cpuset.cpu_exclusive
/bin/echo 0 > /sys/fs/cgroup/cpuset/pub_cpuset/cpuset.mems
