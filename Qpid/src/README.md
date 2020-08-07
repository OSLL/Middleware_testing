# Qpid

## Установка Qpid

Склонировать [репозиторий](https://github.com/apache/qpid-cpp)
```
mkdir build & cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make install
```

## Сборка

```
mkdir <folder_name> && cd <folder_name>
cmake ..
make
```

## Запуск

Запустить сервер:```qpidd --tcp-nodelay --max-connections 0 --ha-flow-messages 0 --session-max-unacked 10000 --default-queue-limit 0```
Через ключ ```-p``` задаётся порт. По умолчанию: ```5672```

Запустить ```QpidPubSub```. Ключи:
* -t, --type - тип узла: publisher, subscriber или ping_pong
* -c, --config - конфигурационный файл для тестов
* -a,--address - адрес брокера(значение поля listener в конфиге брокера)
