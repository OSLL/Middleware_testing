# ActiveMQ

Для сборки ActiveMQ и PubSub необходима утилита maven

## Установка

* Скачать [ActiveMQ Artemis](http://activemq.apache.org/components/artemis/download/)
* Зайти в директорию и выполнить команду ```mvn -Prelease install```

## Сборка

```
mvn generate-sources
mvn package
```

## Запуск

* Установить переменную окружения ```export ACTIVEMQ_ARTEMIS_DIR=/path/to/activemq/artemis/dir```
* Если ActiveMQ собран из исходных файлов, то запустить скрипт ```create_server_source.sh```, иначе ```create_server_bin.sh```
* В файле ```server/etc/broker.xml``` находится настройки сервера
* Запустить сервер ```./server/bin/artemis-service start```
* ```java -jar target/ActiveMQPubSub-1.0.0.jar```. Ключи:
    * -t, --type - тип узла: publisher, sunscriber, ping_pong
    * -c, --config - конфигурационный файл для теста
    * -a, --address - адрес брокера(значение поля listener в конфиге брокера)
    * --first - параметр для первого в ping_pong тесте
