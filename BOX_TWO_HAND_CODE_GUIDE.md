# Разбор кода: коробка, YOLO на сервере, глубина RealSense, две руки

Этот файл объясняет текущую рабочую схему в проекте `ARMS`: робот видит коробку камерой, отправляет картинку на сервер с YOLO, сам считает глубину по bbox, строит точки для двух рук, проверяет обратную кинематику, проверяет препятствия и только потом двигает руки.

## Самые важные команды

Перенос обновленных файлов на робота с ноутбука:

```powershell
scp C:\Users\user\Documents\ChatGPT\ARMS\arms\task7_two_hand_box.py `
    C:\Users\user\Documents\ChatGPT\ARMS\arms\task8_detect_box_and_plan.py `
    C:\Users\user\Documents\ChatGPT\ARMS\run_box_detect_and_plan_fixed_dds.sh `
    unitree@192.168.1.103:~/ARMS_update_tmp/
```

Установка файлов на роботе:

```bash
cd ~/ARMS
mv ~/ARMS_update_tmp/task7_two_hand_box.py arms/task7_two_hand_box.py
mv ~/ARMS_update_tmp/task8_detect_box_and_plan.py arms/task8_detect_box_and_plan.py
mv ~/ARMS_update_tmp/run_box_detect_and_plan_fixed_dds.sh run_box_detect_and_plan_fixed_dds.sh
chmod +x run_box_detect_and_plan_fixed_dds.sh
```

Сухая проверка без движения:

```bash
cd ~/ARMS
EXECUTE=0 MAX_ERROR=0.045 ./run_box_detect_and_plan_fixed_dds.sh
```

Реальный запуск:

```bash
cd ~/ARMS
EXECUTE=1 MAX_ERROR=0.045 ./run_box_detect_and_plan_fixed_dds.sh
```

Возврат рук назад по сохраненной траектории:

```bash
cd ~/ARMS
./restore_both_arms_home_fixed_dds.sh
```

Аварийный прямой возврат в сохраненное состояние:

```bash
cd ~/ARMS
./restore_both_arms_home_direct_fixed_dds.sh
```

## Общая логика

Путь данных такой:

1. `run_box_detect_and_plan_fixed_dds.sh` собирает все настройки и запускает Python.
2. `arms/task8_detect_box_and_plan.py` запускает детектор коробки.
3. `arms/server_box_detector.py` берет кадр с камеры робота, отправляет фото на сервер, получает bbox коробки.
4. `server_box_detector.py` по bbox берет глубину из RealSense и получает координаты коробки в системе камеры: `X`, `Y`, `Z`.
5. `task8_detect_box_and_plan.py` передает эти координаты в `arms/task7_two_hand_box.py`.
6. `task7_two_hand_box.py` переводит координаты из камеры в систему корпуса робота.
7. `task7_two_hand_box.py` строит две цели: левую ладонь к левой стороне коробки, правую ладонь к правой стороне коробки.
8. `arms/urdf_kinematics.py` решает обратную кинематику: какие 7 углов нужны каждой руке.
9. `arms/depth_obstacles.py` строит облако препятствий из RealSense и проверяет путь.
10. Если `EXECUTE=0`, код только печатает план.
11. Если `EXECUTE=1`, код двигает руки, открывает пальцы перед финальным подходом и закрывает пальцы после подведения.

## Главный shell-скрипт

Файл: `run_box_detect_and_plan_fixed_dds.sh`

Это входная точка для демонстрации. Его задача - не делать математику, а собрать параметры и запустить основной Python-код.

Главные переменные:

```bash
EXECUTE="${EXECUTE:-0}"
PALM_UP_OFFSET="${PALM_UP_OFFSET:-0.06}"
HAND_SEQUENCE="${HAND_SEQUENCE:-open-close}"
GRASP_METHOD="${GRASP_METHOD:-wrap}"
OBSTACLE_CLEARANCE="${OBSTACLE_CLEARANCE:-0.10}"
OBSTACLE_EXCLUDE_TARGET_RADIUS="${OBSTACLE_EXCLUDE_TARGET_RADIUS:-0.35}"
```

Что они значат:

- `EXECUTE=0` - только расчет, робот не двигается.
- `EXECUTE=1` - реальное движение.
- `PALM_UP_OFFSET=0.06` - на сколько выше центра коробки целиться ладонями. Если руки идут слишком высоко, уменьшаем. Если слишком низко, увеличиваем.
- `HAND_SEQUENCE=open-close` - включить раскрытие/закрытие пальцев.
- `GRASP_METHOD=wrap` - закрывать кисть методом обхвата.
- `OBSTACLE_CLEARANCE=0.10` - минимальное расстояние от руки до препятствий.
- `OBSTACLE_EXCLUDE_TARGET_RADIUS=0.35` - радиус вокруг коробки, который не считаем препятствием. Это важно, потому что иначе сама коробка и стол около нее могут блокировать подход.

Почему есть DDS-wrapper:

```bash
DDS_WRAPPER="/home/unitree/cyclonedds_py_0_10_fix/run_with_fixed_dds.sh"
```

Unitree SDK2 работает через DDS. На роботе уже была проблема `DDS_RETCODE_PRECONDITION_NOT_MET`; этот wrapper запускает Python в правильной среде, чтобы топики SDK2 создавались нормально.

## Оркестратор коробки

Файл: `arms/task8_detect_box_and_plan.py`

Это связка: сначала детекция, потом планирование.

### `parse_args()`

Создает все аргументы командной строки.

Главные группы аргументов:

- параметры сервера: `--server-url`, `--target-class`;
- параметры камеры: `--camera-mode`, `--depth-width`, `--depth-height`;
- параметры глубины: `--depth-stat`, `--depth-percentile`, `--min-depth`, `--max-depth`;
- параметры коробки: `--box-width`, `--box-height`, `--box-depth`;
- параметры хвата: `--palm-up-offset`, `--palm-forward-offset`, `--hand-sequence`, `--grasp-method`;
- параметры препятствий: `--obstacle-check`, `--obstacle-clearance`, `--obstacle-exclude-target-radius`;
- параметры запуска: `--execute`, `--max-error`.

### `run(args)`

Главная функция файла.

Она делает две команды:

1. Команда детекции:

```python
detect_cmd = [
    sys.executable,
    "-B",
    "-m",
    "arms.server_box_detector",
    ...
]
```

Она запускает `server_box_detector.py`.

2. Команда планирования:

```python
plan_cmd = [
    sys.executable,
    "-B",
    "-m",
    "arms.task7_two_hand_box",
    ...
]
```

Она запускает `task7_two_hand_box.py`.

Если `--execute` пришел в `task8`, он добавляет `--execute` и в `task7`.

### `_load_detection(path)`

Читает JSON, который сохранил детектор:

```json
{
  "camera_point": {
    "x_m": ...,
    "y_m": ...,
    "z_m": ...
  }
}
```

Эти `x_m`, `y_m`, `z_m` - координаты коробки относительно камеры.

## Детектор коробки через сервер

Файл: `arms/server_box_detector.py`

Этот файл нужен потому, что модель YOLO живет на сервере, а глубина есть на роботе.

### Главная идея

Сервер хорошо умеет сказать: “коробка на картинке вот в этом прямоугольнике”.

Робот хорошо умеет сказать: “в этом прямоугольнике глубина примерно 0.45 м”.

Поэтому:

1. робот берет RGB-кадр;
2. отправляет кадр на сервер `/stream`;
3. читает результат `/context`;
4. берет bbox коробки;
5. внутри bbox смотрит глубину RealSense;
6. считает `X`, `Y`, `Z` коробки.

### `ServerBoxDetection`

Это структура результата:

```python
class_name: str
confidence: float
frame: int
x_m: float
y_m: float
z_m: float
bbox: Tuple[float, float, float, float]
source: str
```

Смысл:

- `class_name` - какой класс нашел YOLO, сейчас `box`;
- `confidence` - уверенность модели;
- `bbox` - прямоугольник на RGB-картинке;
- `x_m`, `y_m`, `z_m` - координаты коробки относительно камеры.

### `run(args)`

Главная функция детектора.

Она вызывает:

- `detect_box_from_server_realsense`, если используется RealSense;
- `detect_box_from_server_opencv`, если обычная OpenCV-камера.

Если коробка найдена, сохраняет:

- `runs/server_box_latest.json` - численные координаты;
- `runs/server_box_latest.jpg` - картинка с рамкой;
- `runs/server_box_sent_latest.jpg` - картинка, которую робот отправил серверу;
- `runs/server_box_depth_heatmap_latest.jpg` - тепловая карта глубины.

### `detect_box_from_server_realsense(args)`

Это основной режим.

Шаги:

1. Запускает RealSense color+depth.
2. Читает intrinsics камеры:

```python
intr.fx
intr.fy
intr.ppx
intr.ppy
```

3. Берет RGB-кадр и depth-кадр.
4. Отправляет RGB на сервер через `post_frame`.
5. Читает ответ сервера через `read_fresh_context`.
6. Получает bbox через `detection_from_context`.
7. Считает глубину внутри bbox через `depth_in_bbox`.
8. Пересчитывает пиксель bbox в метры:

```python
x_m = (u - intr.ppx) * z_m / intr.fx
y_m = (v - intr.ppy) * z_m / intr.fy
```

Где:

- `u` - горизонтальная координата центра bbox в пикселях;
- `v` - вертикальная координата центра bbox в пикселях;
- `intr.ppx` - центр камеры по X в пикселях;
- `intr.ppy` - центр камеры по Y в пикселях;
- `intr.fx`, `intr.fy` - фокусные расстояния камеры в пикселях;
- `z_m` - глубина до коробки в метрах.

### `post_frame(server_url, frame)`

Отправляет фото на сервер.

Важно: сервер может принимать кадры и от ноутбука, и от робота. Поэтому мы добавили защиту `context-frame-match=exact`.

### `read_fresh_context(...)`

Читает ответ сервера, но не любой, а свежий ответ именно на тот кадр, который отправил робот.

Если этого не делать, можно получить bbox от чужого кадра, например от ноутбука.

### `detection_from_context(...)`

Достает из ответа сервера объект нужного класса.

Если `target_class=box`, функция ищет именно коробку.

### `depth_in_bbox(...)`

Берет глубину не из одной точки, а из области внутри bbox.

Сейчас важные настройки:

- `depth_roi_ratio=0.45` - берем центральную часть bbox, а не весь прямоугольник;
- `depth_stat=nearest_percentile` - берем ближний процентиль, а не медиану;
- `depth_percentile=20` - берем 20-й процентиль глубины.

Это сделано потому, что у коробки может быть задняя стенка, передняя стенка, пустота внутри. Нам чаще нужна ближняя полезная поверхность.

## Планировщик двух рук

Файл: `arms/task7_two_hand_box.py`

Это главный файл для рук.

### `BoxTargets`

Хранит 4 точки:

```python
detected_center
grasp_center
left
right
```

Смысл:

- `detected_center` - реальный центр коробки по камере после перевода в координаты корпуса;
- `grasp_center` - центр, от которого строим хват;
- `left` - цель для левой ладони;
- `right` - цель для правой ладони.

### `LEFT_SOFT_PREGRASP`

Это мягкая подготовительная поза для левой руки.

Она зеркалит `RIGHT_SOFT_PREGRASP` из `task5_ik_reach.py`.

Идея: перед финальным подходом руки сначала уходят в безопасную форму, а не сразу летят из домашнего положения к коробке.

### `parse_args()`

Создает аргументы для планировщика.

Самые важные:

- `--camera-x/y/z` - координаты коробки от камеры;
- `--torso-x/y/z` - можно задать координаты сразу от корпуса, без камеры;
- `--box-width` - ширина коробки;
- `--side-clearance` - насколько ладони стоят снаружи коробки;
- `--palm-forward-offset` - смещение ладоней вперед/назад;
- `--palm-up-offset` - высота ладоней относительно центра коробки;
- `--hand-sequence` - включить или выключить пальцы;
- `--grasp-method` - способ закрытия пальцев;
- `--execute` - реально двигать или только считать;
- `--obstacle-check` - проверять препятствия или нет.

### `run(args)`

Главная функция.

Она идет по этапам:

1. `build_box_targets(args)` - строит цели для двух ладоней.
2. `load_urdf(args.urdf)` - загружает модель робота.
3. `path_to_link(...)` - строит цепочку суставов от корпуса до ладони.
4. `solve_hand_target(...)` - решает IK для левой руки.
5. `solve_hand_target(...)` - решает IK для правой руки.
6. `save_report(...)` - сохраняет отчет.
7. `print_report(...)` - печатает главные числа в терминал.
8. Проверяет IK ошибку.
9. Строит путь:

```python
left_path = [left_seed, left_seed, left_q]
right_path = [right_seed, right_seed, right_q]
```

10. Если включены препятствия, вызывает `check_two_hand_obstacles`.
11. Если `execute` не включен, завершает сухой прогон.
12. Если `execute` включен:
    - создает контроллер рук;
    - сохраняет snapshot;
    - сохраняет путь в `runs/box_two_hand_last_path.json`;
    - двигает руки по waypoint;
    - раскрывает пальцы перед финальным подходом;
    - закрывает пальцы после финального подхода.

### Почему пальцы открываются не сразу

Раньше было опасно: если открыть пальцы в домашней позе, кисть может задеть ноги.

Теперь код делает так:

```python
if args.hand_sequence == "open-close" and not hands_opened and index == final_waypoint_index:
    dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    dex3.open("left")
    dex3.open("right")
```

То есть:

1. сначала руки идут в промежуточную позу;
2. только перед последним движением пальцы открываются;
3. потом руки идут к коробке;
4. потом пальцы закрываются.

### `build_box_targets(args)`

Строит точки для ладоней.

Формула:

```python
side_offset = box_width / 2 + side_clearance

left = PoseXYZ(
    x = grasp_center.x + palm_forward_offset,
    y = grasp_center.y + side_offset,
    z = grasp_center.z + palm_up_offset,
)

right = PoseXYZ(
    x = grasp_center.x + palm_forward_offset,
    y = grasp_center.y - side_offset,
    z = grasp_center.z + palm_up_offset,
)
```

Если руки берут слишком высоко:

- уменьшать `PALM_UP_OFFSET`;
- сейчас стоит `0.06`.

Если руки слишком далеко вперед/назад:

- менять `PALM_FORWARD_OFFSET`;
- сейчас стоит `-0.02`.

Если руки слишком широко/узко:

- менять `BOX_WIDTH`;
- менять `SIDE_CLEARANCE`.

### `choose_grasp_center(args, detected_center)`

Решает, какой `y` использовать для двух рук.

Есть три режима:

- `detected` - брать реальное положение коробки;
- `zero` - хват строится по центру робота;
- `clamp` - ограничить реальное положение.

Сейчас стоит `zero`, потому что для двух рук проще держать симметричный хват относительно корпуса.

### `read_box_center(args)`

Если передали `--torso-x/y/z`, берет их напрямую.

Если передали `--camera-x/y/z`, вызывает:

```python
yolo_camera_point_to_arm_target(...)
```

Это переводит точку из системы камеры в систему корпуса.

### `solve_hand_target(...)`

Решает обратную кинематику для одной руки.

Вход:

- цепочка суставов;
- целевая точка ладони;
- начальное приближение суставов;
- число итераций.

Выход:

- `q` - 7 углов руки;
- `error` - насколько ладонь не попала в цель;
- `pos` - куда реально пришла ладонь по FK.

Если `error > MAX_ERROR`, движение запрещается.

### `check_two_hand_obstacles(...)`

Проверяет путь на препятствия.

Логика:

1. Получает облако препятствий из RealSense.
2. Сохраняет его в `runs/box_obstacle_cloud_latest.json`.
3. Строит кандидаты маршрутов.
4. Для каждого кандидата проверяет левую и правую руку.
5. Выбирает самый безопасный путь.

Важные числа:

- `OBSTACLE_CLEARANCE=0.10` - минимальное расстояние до препятствий;
- `OBSTACLE_EXTRA_CLEARANCE=0.00` - дополнительный запас;
- `OBSTACLE_EXCLUDE_TARGET_RADIUS=0.35` - не считать саму коробку препятствием;
- `OBSTACLE_EARLY_ACCEPT_DISTANCE=0.20` - если найден очень хороший путь, можно принять его раньше.

Если видишь:

```text
BOX_OBSTACLE_CLEAR
```

Значит маршрут найден.

Если видишь:

```text
BOX_NO_SAFE_PATH
```

Значит маршрут заблокирован проверкой препятствий.

### `build_two_hand_path_candidates(...)`

Генерирует разные варианты маршрута.

Он перебирает:

- насколько поднять руки вверх;
- насколько развести в стороны;
- насколько сместить вперед/назад.

Из этих вариантов получаются названия вроде:

```text
attempt2_via_up+0.27_side+0.09_forward-0.24
```

Это читается так:

- `attempt2` - вторая попытка перепланирования;
- `via_up+0.27` - промежуточная точка на 27 см выше;
- `side+0.09` - руки разведены еще на 9 см;
- `forward-0.24` - промежуточная точка смещена назад/ближе на 24 см по оси X.

### `solve_waypoint_or_none(...)`

Проверяет, достижима ли промежуточная точка.

Если IK ошибка больше `AVOIDANCE_MAX_ERROR`, этот waypoint выкидывается.

### `save_snapshot(...)`

Сохраняет стартовое положение рук перед движением.

Это нужно для восстановления.

### `save_motion_path(...)`

Сохраняет реальный путь:

```json
{
  "left_path": [...],
  "right_path": [...]
}
```

Потом `restore_both_arms_home_fixed_dds.sh` может пройти этот путь обратно.

## Кисти Dex3

Файлы:

- `arms/dex3_hand.py`
- `arms/dex3_sdk2_backend.py`

### `Dex3Pose`

Это набор 7 чисел для кисти:

- `thumb_abduction` - поворот большого пальца в сторону;
- `thumb_flexion_0` - сгиб в основании большого пальца;
- `thumb_flexion_1` - сгиб фаланги большого пальца;
- `index_flexion_0` - первый сустав указательного;
- `index_flexion_1` - второй сустав указательного;
- `middle_flexion_0` - первый сустав среднего;
- `middle_flexion_1` - второй сустав среднего.

### `Dex3HandController.open(hand)`

Отправляет открытую позу кисти.

### `Dex3HandController.close(hand, method)`

Закрывает кисть выбранным методом.

Для коробки сейчас используется:

```bash
GRASP_METHOD=wrap
```

### `UnitreeSdk2Dex3Backend`

Это слой, который превращает нормальные имена пальцев в массив `q` для Unitree SDK2 и отправляет команду в DDS-топик кисти:

- `rt/dex3/left/cmd`;
- `rt/dex3/right/cmd`.

## Кинематика URDF

Файл: `arms/urdf_kinematics.py`

Этот файл делает прямую и обратную кинематику.

### `load_urdf(path)`

Читает URDF модель робота.

URDF описывает:

- звенья;
- суставы;
- оси вращения;
- ограничения суставов;
- смещения между звеньями.

### `path_to_link(model, base, tip)`

Находит цепочку суставов от `torso_link` до ладони.

Для правой руки:

```text
torso_link -> right_hand_palm_link
```

Для левой:

```text
torso_link -> left_hand_palm_link
```

### `fk(chain, values_by_name)`

Forward Kinematics, прямая кинематика.

Вход: углы суставов.

Выход: положение ладони в пространстве.

То есть вопрос:

> Если суставы стоят вот так, где находится ладонь?

### `solve_ik_position(...)`

Inverse Kinematics, обратная кинематика.

Вход: желаемая точка ладони.

Выход: углы суставов.

То есть вопрос:

> Какие углы суставов нужны, чтобы ладонь пришла сюда?

### Почему бывает IK error

IK может не попасть точно, если:

- точка физически недостижима;
- цель слишком далеко;
- цель слишком низко/высоко;
- суставы уперлись в лимиты;
- начальное приближение плохое.

В логах:

```text
LEFT_IK_ERROR_M 0.010
RIGHT_IK_ERROR_M 0.010
```

Это хорошо.

А вот так плохо:

```text
BOX_IK_BLOCKED
left_ik_error_m=0.112
```

## Препятствия и облако точек

Файл: `arms/depth_obstacles.py`

### Что такое облако точек

Облако точек - это много 3D-точек перед камерой:

```text
(x, y, z)
(x, y, z)
(x, y, z)
...
```

Каждая точка говорит: “вот тут в пространстве есть поверхность”.

Для нас это поверхности стола, коробки, стульев, человека, пола и всего, что RealSense видит.

### `capture_realsense_obstacle_cloud(...)`

Снимает depth-картинку и превращает ее в точки в системе корпуса робота.

При этом точка рядом с коробкой исключается:

```python
exclude_target_radius_m=args.obstacle_exclude_target_radius
```

Иначе сама коробка будет считаться препятствием, и робот не сможет к ней подойти.

### `camera_point_to_torso(...)`

Переводит 3D-точку из камеры в координаты корпуса.

Использует тот же transform, что и для коробки:

- положение камеры относительно корпуса;
- угол наклона камеры;
- знаки осей.

### `voxel_downsample(...)`

Уменьшает число точек.

Камера может дать тысячи точек. Проверять каждую дорого. Поэтому близкие точки объединяются в “ячейки” размером `voxel_size_m`.

### `check_joint_path_collision(...)`

Проверяет путь руки.

Что делает:

1. Берет путь суставов.
2. Разбивает движение на маленькие шаги.
3. Для каждого шага считает положение сегментов руки.
4. Проверяет расстояние от сегментов руки до точек препятствий.
5. Если расстояние меньше `clearance_m`, путь запрещается.

Пример плохого лога:

```text
collision risk: segment=arm_segment_6 sample=5 distance=0.110m clearance=0.120m
```

Это значит:

- проверялся сегмент руки `arm_segment_6`;
- на шаге `sample=5`;
- ближайшее препятствие было в 11 см;
- требовалось минимум 12 см;
- маршрут заблокирован.

## Восстановление рук

Файл: `arms/restore_two_hand_path.py`

### `run(args)`

Восстанавливает руки назад.

Если есть `runs/box_two_hand_last_path.json`, он берет путь и идет по нему в обратном порядке.

Это лучше, чем просто резко вернуть руки домой, потому что обратный путь повторяет уже пройденную траекторию.

### `validate_path(...)`

Проверяет, что путь выглядит правильно:

- это список;
- в каждой точке 7 чисел;
- есть минимум две точки.

### `load_snapshot(...)`

Если путь потерян, можно взять snapshot из `before_box_two_hand.json`.

## Как читать главные логи

### Детекция

Хороший пример:

```text
BOX_DETECTION {"class_name": "box", "confidence": 0.457, "x_m": 0.040, "y_m": -0.008, "z_m": 0.328}
```

Смысл:

- коробка найдена;
- уверенность 0.457;
- координаты относительно камеры получены.

Плохой пример:

```text
SERVER_BOX no detection target_class=box
```

Смысл: сервер не нашел коробку на кадре робота.

### Цели рук

```text
LEFT_TARGET_TORSO {"x_m": ..., "y_m": ..., "z_m": ...}
RIGHT_TARGET_TORSO {"x_m": ..., "y_m": ..., "z_m": ...}
```

Это точки, куда должны прийти ладони.

### IK

Хорошо:

```text
LEFT_IK_ERROR_M 0.007
RIGHT_IK_ERROR_M 0.007
```

Плохо:

```text
BOX_IK_BLOCKED
```

### Препятствия

Хорошо:

```text
BOX_OBSTACLE_CLEAR path=...
```

Плохо:

```text
BOX_NO_SAFE_PATH
```

### Реальное движение

Когда `EXECUTE=1`, должны появиться строки:

```text
BOX_TWO_HAND execute waypoint 1/2
BOX_TWO_HAND hand sequence 1/2: open both hands before final approach
BOX_TWO_HAND execute waypoint 2/2
BOX_TWO_HAND hand sequence 2/2: close both hands method=wrap
```

Если этих строк нет, значит движение не началось.

## Что менять при типичных проблемах

### Руки слишком высоко

Менять:

```bash
PALM_UP_OFFSET=0.04
```

или ниже:

```bash
PALM_UP_OFFSET=0.02
```

Сейчас дефолт:

```bash
PALM_UP_OFFSET=0.06
```

### Руки слишком низко

Увеличить:

```bash
PALM_UP_OFFSET=0.08
```

### Руки не начинают двигаться, пишет `BOX_NO_SAFE_PATH`

Для проверки можно уменьшить зазор:

```bash
OBSTACLE_CLEARANCE=0.08
```

Или увеличить область вокруг коробки, которую не считаем препятствием:

```bash
OBSTACLE_EXCLUDE_TARGET_RADIUS=0.40
```

Осторожно: слишком маленький `OBSTACLE_CLEARANCE` делает движение менее безопасным.

### IK ошибка слишком большая

Если видишь:

```text
BOX_IK_BLOCKED
```

Нужно смотреть:

- коробка слишком далеко?
- коробка слишком низко?
- `PALM_UP_OFFSET` слишком большой?
- `BOX_WIDTH` слишком большой?
- `SIDE_CLEARANCE` слишком большой?

### Сервер видит не тот кадр

Проверить:

```bash
CONTEXT_FRAME_MATCH=exact
```

Это уже стоит по умолчанию.

Также смотреть:

```text
runs/server_box_sent_latest.jpg
```

Это именно кадр, который робот отправил серверу.

## Минимальная схема для восстановления с нуля

Если надо самому воспроизвести систему, порядок такой:

1. На роботе должна работать камера RealSense.
2. На сервере должен работать сервис с YOLO и эндпоинтами `/stream`, `/context`.
3. Робот запускает `server_box_detector.py`.
4. Детектор получает bbox коробки.
5. Детектор берет глубину из RealSense внутри bbox.
6. `task8_detect_box_and_plan.py` передает координаты в `task7_two_hand_box.py`.
7. `task7_two_hand_box.py` строит левую и правую цель.
8. `urdf_kinematics.py` решает IK.
9. `depth_obstacles.py` проверяет путь.
10. При `EXECUTE=1` руки двигаются, кисти открываются перед финальным подходом и закрываются после подхода.

## Текущие важные значения

```bash
PALM_UP_OFFSET=0.06
HAND_SEQUENCE=open-close
GRASP_METHOD=wrap
OBSTACLE_CLEARANCE=0.10
OBSTACLE_EXCLUDE_TARGET_RADIUS=0.35
MAX_ERROR=0.045
CAMERA_PITCH_DOWN_DEG=42
DEPTH_STAT=nearest_percentile
DEPTH_PERCENTILE=20
CONTEXT_FRAME_MATCH=exact
```

Это текущая рабочая база для тестов с коробкой.
