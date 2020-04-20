## RabbitMq

### Зависимости

Для сборки и запуска необходимы:
* [RabbitMQ](https://www.rabbitmq.com/download.html)
* [SimpleAmqpClient](https://github.com/alanxz/SimpleAmqpClient)

### Сборка
    mkdir <folder_name> && cd <folder_name>
    cmake ..
    make

### Запуск

    ./pub <config_for_test> <config_for_connection>
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
