# Анализ версии ARMS, скачанной с робота 2026-08-25

## Где лежит скачанная версия

Полный архив с робота:

```text
C:\Users\user\Documents\ChatGPT\ARMS\backups\ARMS_robot_20260825_121321.tar.gz
```

Распакованный полный снимок:

```text
C:\Users\user\Documents\ChatGPT\ARMS\robot_snapshots\ARMS_robot_20260825_121321\ARMS
```

Чистая версия для GitHub:

```text
C:\Users\user\Documents\ChatGPT\ARMS\github_ready\ARMS_robot_20260825_clean
```

Архив чистой версии для GitHub:

```text
C:\Users\user\Documents\ChatGPT\ARMS\backups\ARMS_robot_20260825_clean_github.zip
```

## Что было сделано

1. С робота `unitree@192.168.1.103` скачан проект `~/ARMS`.
2. Сделан полный сырой бэкап без удаления логов и rollback-файлов.
3. Проект распакован отдельно для анализа, без перезаписи локальной рабочей папки.
4. Собрана отдельная GitHub-ready копия.
5. Из GitHub-ready копии удалены:
   - большие `.log`;
   - `runs/`;
   - `backups/`;
   - `_camera_debug/`;
   - `rollback_current_*`;
   - `__pycache__/`;
   - `.bak*`;
   - временные картинки и мусорные файлы.
6. В GitHub-ready копию добавлен `.gitignore`.
7. В GitHub-ready копию добавлены документы:
   - `BOX_TWO_HAND_CODE_GUIDE.md`;
   - `SERVER_YOLO_START_GUIDE.md`.
8. Проверена компиляция Python-файлов.
9. Прогнаны тесты.

## Результат тестов

Команда:

```bash
python -m pytest -q --basetemp C:\Users\user\Documents\ChatGPT\ARMS\github_ready\ARMS_robot_20260825_clean\tmp\pytest_basetemp
```

Результат:

```text
10 passed in 0.13s
```

Была одна маленькая несовместимость теста с `task5_ik_reach.py`: тестовый `SimpleNamespace` не содержал `avoidance_skip_direct`.

Исправлено в GitHub-ready копии:

```python
skip_direct = bool(getattr(args, "avoidance_skip_direct", False))
```

Это безопасно: если аргумент есть, код работает по нему; если старый вызов аргумент не передал, используется `False`.

## Главное отличие версии с робота от старой локальной версии

Роботская версия существенно новее.

В ней уже есть:

- двухручный сценарий коробки;
- серверная YOLO-детекция коробки;
- RealSense depth по bbox;
- фильтры свежести bbox/context;
- защита от чужого или старого кадра через `context-frame-match=exact`;
- проверка препятствий по облаку точек;
- построение маршрутов обхода;
- hand-envelope: проверка раскрытой кисти как объема, а не только линии руки;
- стратегия `flat-squeeze`;
- отдельные probe-скрипты для большого пальца и передних пальцев;
- DDS fixed wrapper для Unitree SDK2;
- возврат рук по сохраненному пути.

## Главный запуск коробки

Файл:

```text
run_box_detect_and_plan_fixed_dds.sh
```

Текущие важные значения:

```bash
PALM_UP_OFFSET=0.075
HAND_SEQUENCE=open-only
GRASP_METHOD=flat_palm
GRASP_STRATEGY=flat-squeeze
SQUEEZE_SIDE_CLEARANCE=-0.015
OBSTACLE_CLEARANCE=0.07
OBSTACLE_EXTRA_CLEARANCE=0.03
HAND_ENVELOPE_LENGTH=0.10
HAND_ENVELOPE_WIDTH=0.09
HAND_ENVELOPE_CLEARANCE=0.03
CONTEXT_FRAME_MATCH=exact
```

Смысл:

- руки идут к коробке двумя ладонями;
- пальцы/ладони приводятся в плоское состояние;
- потом выполняется легкий squeeze;
- раскрытая кисть учитывается в проверке препятствий;
- детекция должна быть свежей, от кадра робота.

## Реальный запуск на роботе

```bash
cd ~/ARMS
EXECUTE=1 MAX_ERROR=0.045 ./run_box_detect_and_plan_fixed_dds.sh
```

Сухой запуск без движения:

```bash
cd ~/ARMS
EXECUTE=0 MAX_ERROR=0.045 ./run_box_detect_and_plan_fixed_dds.sh
```

Возврат рук назад:

```bash
cd ~/ARMS
./restore_both_arms_home_fixed_dds.sh
```

## Возврат рук

Файл:

```text
restore_both_arms_home_fixed_dds.sh
```

Он вызывает:

```bash
python3 -B -m arms.restore_two_hand_path
```

Использует:

```bash
PATH_FILE=runs/box_two_hand_last_path.json
SNAPSHOT_FILE=before_box_two_hand.json
HOME_SNAPSHOT_FILE=baseline_home_snapshot.json
```

Это значит:

- если есть последний путь, руки идут обратно по нему;
- если пути нет, есть fallback на сохраненный snapshot;
- DDS запускается через isolated fixed wrapper.

## Сервер YOLO

Документ:

```text
SERVER_YOLO_START_GUIDE.md
```

Главная проблема, которую мы нашли:

```text
agrohab-app был запущен с DETECT=0
```

Из-за этого `/context` не обновлялся:

```text
frame_seq=0
objects=[]
```

Правильно:

```bash
DETECT=1
DETECTOR_URL=http://agrohab-detector:8000
```

YOLO detector должен быть отдельным контейнером:

```text
agrohab-detector на 8001
```

## Что НЕ надо выкладывать в GitHub

Не выкладывать:

- `*.log`;
- `runs/`;
- `_camera_debug/`;
- `rollback_current_*`;
- `backups/`;
- `__pycache__/`;
- `*.bak*`;
- временные json-снимки вида `before_*.json`, если они сняты с реального робота и не нужны как пример.

Почему:

- логи большие;
- там могут быть IP, локальные пути, временные состояния робота;
- rollback-файлы шумят и мешают понять актуальную версию.

## Что стоит выложить

Из GitHub-ready копии стоит выложить:

- `arms/`;
- `config/`;
- `tests/`;
- `run_box_detect_and_plan_fixed_dds.sh`;
- `restore_both_arms_home_fixed_dds.sh`;
- `restore_both_arms_home_direct_fixed_dds.sh`;
- `restore_right_arm_home_fixed_dds.sh`;
- `run_task6_detect_and_reach_fixed_dds.sh`;
- probe-скрипты `run_right_*_fixed_dds.sh`;
- `README.md`;
- `REAL_TEST_COMMANDS.md`;
- `FULL_CUP_TEST_RUNBOOK.md`;
- `BOX_TWO_HAND_CODE_GUIDE.md`;
- `SERVER_YOLO_START_GUIDE.md`;
- `.gitignore`.

## Что проверить перед публикацией

1. Открыть GitHub-ready папку:

```text
C:\Users\user\Documents\ChatGPT\ARMS\github_ready\ARMS_robot_20260825_clean
```

2. Проверить, что там нет больших логов.
3. Проверить, что нет секретов кроме локальных IP робота/сервера.
4. Если репозиторий будет публичным, подумать, надо ли заменить IP:

```text
192.168.1.103
192.168.10.179
```

на переменные окружения в README.

5. Запустить тесты:

```bash
python -m pytest -q --basetemp tmp/pytest_basetemp
```

## Итог

Версия с робота сохранена полностью и отдельно подготовлена для GitHub.

Самое важное:

- полный бэкап не тронут;
- чистая версия собрана;
- тесты проходят;
- GitHub-ready архив создан.
