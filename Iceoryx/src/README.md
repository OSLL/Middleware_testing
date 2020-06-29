## Iceoryx

### Зависимости

* [Iceoryx](https://github.com/eclipse/iceoryx)
   * Следующая команда установит необходимые пакеты ```sudo apt install cmake libacl1-dev libncurses5-dev pkg-config```
   * Также для работы **RouDi** необходима библиотека [cpptoml](https://github.com/skystrife/cpptoml).
 Её может установить скрипт, так что ручная установка не обязательна

### Установка Iceoryx

1. Склонировать репозиторий
2. Запустить скрипт ```./tools/iceoryx_build_test.sh```, передав параметры *clean* и *release*
3. Скопировать все файлы из папки ```build/install/prefix/include``` в папку с заголовками
4. Скопировать все библиотеки из папки ```build/install/prefix/lib``` в папку с библиотеками
5. (Не обязательный шаг) Скопировать из папки ```build/install/prefix/bin``` RouDi и стандартный конфигурационный файл
для него из папки ```build/install/prefix/etc``` в удобное для запуска место

### Сборка

```
mkdir <folder_name> && cd <folder_name>
cmake ..
make
```

### Запуск

* Запустить RouDi, через ключ ```-c``` передаётся абсолютный путь к config-файлу
    * Если при запуске возникла ошибка ```Bus error (core dumped)```, то в файле ```roudi_config.toml``` уменьшить значения count 
* Запустить ```./PubSub```
    * Ключ -t(--type) - тип узла: publisher, subscriber или ping_pong
    * Ключ -c(--config) - конфигурационный файл для тестов
