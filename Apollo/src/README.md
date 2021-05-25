# Apollo (Cyber RT)

## Установка и сборка

[Apollo](https://github.com/ApolloAuto/apollo)

Зависимости можно посмотреть в репозитории _apollo_

В **эту** папку скопировать из репозитория следующие папки:
* ```cyber```
* ```tools```
* ```third_party```
* ```scripts```, только файл ```apollo.bashrc```

Также скопировать файл ```CPPLINT.cfg```. В файле ```scripts/apollo.bashrc``` 
значение переменной ```APOLLO_ROOT_DIR``` изменить на текущую директорию. 
Установить следующие переменные окружения:

* ```DEBIAN_FRONTEND=noninteractive```
* ```PATH=/opt/apollo/sysroot/bin:$PATH```

Из папки ```docker/build``` репозитория скопировать папки в ```/tmp```:
* ```installers```
* ```rcfiles```
* ```archive```

Запустить следующие скрипты:
* ```bash /tmp/installers/install_minimal_environment.sh``` (необязательно, скорее всего всё установленно)
* ```bash /tmp/installers/install_cyber_deps.sh```
* ```bash /tmp/installers/install_bazel.sh```
* ```bash /tmp/installers/post_install.sh cyber``` 

Скопировать заголовочные файлы в ```testing```.
Собрать командой:

```
bazel build --copt="-std=c++17" testing/...
```

После выполнить: ```. cyber/setup.bash```

## Запуск

Запустить ```bazel-bin/testing/PubSub```. Ключи:
* -t, --type - тип узла: publisher, sunscriber, ping_pong
* -c, --config - конфигурационный файл для теста

