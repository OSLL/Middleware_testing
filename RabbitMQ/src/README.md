## RabbitMq

### Зависимости

Для сборки и запуска необходимы:
* [RabbitMQ](https://www.rabbitmq.com/download.html)
* [SimpleAmqpClient](https://github.com/alanxz/SimpleAmqpClient)
  * Для неё необходима библиотека [rabbitmq-c](https://github.com/alanxz/rabbitmq-c)

#### Установка библиотек

Склонировать репозитории.

Для rabbitmq-c:
```
mkdir build && cd build
cmake ..	//задание места установки -DCMAKE_INSTALL_PREFIX=
cmake --build . --config Release --target install
```

Для SimpleAmqpClient:
```
mkdir build && cd build
cmake ..	//задание места установки -DCMAKE_INSTALL_PREFIX=
make install
```

### Сборка
    mkdir <folder_name> && cd <folder_name>
    cmake ..
    make

### Запуск

    ./pub <config_for_connection> <config_for_test>
Для подписчика аналогично

Второй файл типа json для настройки подключения к серверу. Должен в себе содержать следующее:
* host - адрес сервера
* port - порт подключения
* username - login для авторизации
* password - пароль для login
* vhost - виртуальный хост
* max_frame - максимальный размер( в байтах) одного сообщения
* exchange

Пример файла:
```json
{
	"host": "localhost",
	"port": 5672,
	"username": "guest",
	"password": "guest",
	"vhost": "/",
	"max_frame": 131072,
	"exchange": "amq.direct"
}
```

### Возможности RabbitMQ

#### Гарантия доставки

Протокол TCP гарантирует доставку пакета до сервера. Если в методе ```BasicPublish``` третьим аргумент 
передать **true**(mandatory), то издатель будет проверять, дошло ли сообщение до сервера. Если нет, 
то выбрасывается исключение ```MessageReturnedException```.

Сообщению в ```DelivaryMode``` можно выставить значение ```dm_persistent```. Это означает, что на сервере 
сообщение будет хранится на диске, пока не будет отправлено.

#### Остальные настройки

* Возможность использовать протокол SSL
* Возможность сохранения очередей на сервере после перезапуска
* Авто-удаление очередей
* Consumer Acknowledgement и Publisher Confirms(см. Гарантия доставки)
* Три типа exchange:
  * Прямой (Direct)
  * От одного ко всем (Fanout)
  * Топик, гибрид предыдущих двух (Topic)
* Возможность узнать, что произошло не так. Например, потеря соединения, не существование очереди и т.д.
