## ROS2-DDS-Testing

**Запуск тестов.**

Все файлы находятся на ветке test_rate!

* Неодходимо перейти в режим суперпользователся для создания cgroup и добавления в него PID: 
`sudo su`

* Затем запустить скрипт:
`./env_prep.bash

* Теперь можно запустить тесты, для запуска 1 или 3 теста необходимо ввести команду:

`RMW_IMPLEMENTATION=rmw_opensplice_cpp ros2 launch test_sub_and_pub run_test1_3.launch.py message_number:=<number of msgs to be sent> message_length:=<length of each message>`
