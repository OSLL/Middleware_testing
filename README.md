## ROS2-DDS-Testing

**Запуск тестов.**

* Неодходимо перейти в режим суперпользователся для создания cgroup и добавления в него PID: 
`sudo su`

* Затем запустить скрипт:
`./create_cpuset.sh`

* Если не настроено окружение, то сначала натроим его: 
`source /opt/ros/<distro>/setup.bash`

* Перейдем в папку `test_delays` и скомпилируем пакет:

`colcon build`

* Сконфигурируем окружение пакета:
`source ./install/setup.bash`

* Теперь можно запустить тесты, для запуска 1 или 3 теста необходимо ввести команду:

`RMW_IMPLEMENTATION=rmw_opensplice_cpp ros2 launch test_sub_and_pub run_test1_3.launch.py message_number:=<number of msgs to be sent> message_length:=<length of each message>`
