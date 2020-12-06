# YAMI

## Установка

[Yami](http://www.inspirel.com/yami4/)

* Скачивать архив "Universal source package: libraries and services"
* Распаковать архив
* Перейти в папку ```src/core``` и выполнить сборку командой ```make```
* Перейти в папку ```src/cpp``` и выполнить сборку командой ```make```
* Скопировать файлы из ```include``` в папку с заголовочными файлами
* Скопировать файлы из ```lib``` в папку с библиотеками

## Сборка

```
mkdir <folder_name> && cd <folder_name>
cmake ..
make
```

Для ping_pong тестов также нужно собрать yami4names. Для этого нужно
перейти в папку ```<yami_install_dir>/src/services/names``` и ввести ```make```

## Запуск

Для ping_pong тестов нужно предварительно запустить yami4names из папки 
```<yami_install_dir>/src/services/names```

Запустить ```YamiPubSub```. Ключи:
* -t, --type - тип узла: publisher, sunscriber, ping_pong
* -c, --config - конфигурационный файл для теста
* -a, --address - адрес для связи между узлами
* --server - адрес сервера, хранящего информацию о портах для топиков. Необходимо для ping_pong тестов

