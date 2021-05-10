# ActiveMQ

Для сборки ActiveMQ и PubSub необходима утилита maven

## Установка

* Скачать [GlassFish](https://www.eclipse.org/downloads/download.php?file=/glassfish/glassfish-5.1.0.zip)
* Распаковать архив

## Сборка

Установить переменную окружения ```export IMQ_HOME=/path/to/mq/folder``` 

```
./set_maven.sh
mvn generate-sources
mvn package
```

## Запуск

* Запустить сервер: ```$IMQ_HOME/bin/imqbrokerd```
* ```java -jar target/ActiveMQPubSub-1.0.0.jar```. Ключи:
    * -t, --type - тип узла: publisher, sunscriber, ping_pong
    * -c, --config - конфигурационный файл для теста
    * -a, --address - адрес брокера в формате ```mq://<dns_name>:<port>```
    * --first - параметр для первого в ping_pong тесте
