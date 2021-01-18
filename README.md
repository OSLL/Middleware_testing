## Middleware testing

### Зависимости

Для запуска тестов необходим python 3 версии >= 3.6. Установить можно с помощью команды:

`sudo apt-get install python3.6`

Далее необходимо установить python модули, используемые в тестовом скрипте. Для начала надо установить утилиту pip:

`sudo apt-get install pip3`

Далее устанавливаем с помощью pip модули:

`sudo pip3 install numpy matplotlib py-cpuinfo psutil`

### Запуск тестов

Переходим в директорию testing_tool. В 13 и 14 строчки между квадратных скобок добавляем в кавычках пути к исполняемым файлам узлов тестируемых фреймворков. Например:
```
    pubs = ["../FastRTPS/src/build/FastRTPSTest "]
    subs = ["../FastRTPS/src/build/FastRTPSTest "]
```

Необходимо сгенерировать конфигурационные файлы командой:

`python3 generate_configs.py`

Запуск тестов производится следующей командой:

`sudo python3 test.py`

Для построения графиков по полученным данным необходимо запустить скрипт plotting.py:

`python3 plotting.py`

