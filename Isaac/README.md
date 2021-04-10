# Isaac

## Установка Isaac

* Скачать [Isaac](https://developer.nvidia.com/isaac/downloads)
* Распаковать архив
* Запустить скрипт ```./engine/engine/build/scripts/install_dependencies.sh```

## Сборка

В файл ```sdk/WORKSPACE``` после строк 
```
local_repository(
    name = "com_nvidia_isaac_engine",
    path = "../engine",
)
```
добавить строки
```
new_local_repository(
    name = "test_interface",
    path = "path/to/interface/folder",
    build_file = "test_interface.BUILD",
)
```
где в ```path``` указать относительный путь до папки ```interface```

Копировать файл ```test_interface.BUILD``` в папку ```sdk```

Копировать папку ```testing``` в ```sdk/packages/```

Выполнить следующие команды:
```
cd sdk/packages/testing
bazel build testing main
```

## Запуск

* С функицей **main** от разработчиков **Isaac** ```bazel run testing```
* С функцией **main** для тестирования: запускать из папки ```sdk/bazel/bin``` командой 
```./packages/testing/main --app /absolute/path/to/json/file```(через ключ ```--app``` 
передаётся абсолютный путь к json-файлу приложения)

## Для тестирования

В ```pubs``` и ```subs``` добавить ```../Isaac/```, в ```isaac_dir``` записать полный путь
до папки, в которой находится ```sdk```
