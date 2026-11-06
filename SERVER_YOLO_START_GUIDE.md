# Как поднять сервер YOLO для коробки

Этот файл про ошибку:

```text
SERVER_BOX stale/foreign context expected_seq=24 last_seq=0 frame_match=exact
SERVER_BOX no detection target_class=box server=http://192.168.10.179:8000
```

## Что это значит

Робот отправляет кадры на сервер `192.168.10.179:8000`, но серверный `/context` не обновляется.

В логе:

```text
expected_seq=24 last_seq=0
```

Смысл:

- робот отправил кадр номер `24`;
- сервер вернул старый `context` с `frame_seq=0`;
- значит YOLO-обработка на сервере не привязалась к новым кадрам.

RealSense на роботе в этот момент работает. Проблема не в глубине, а в серверной части.

## Что я нашел на сервере

Сервер:

```bash
ssh unitree@192.168.10.179
# Введите пароль сервера вручную при запросе SSH.
```

На сервере есть проект:

```bash
~/agrohab
```

Важные файлы:

```bash
~/agrohab/server/app.py
~/agrohab/deploy/detector/app.py
~/agrohab/deploy/weights/best.pt
~/agrohab/deploy/weights/yolo11s.pt
~/agrohab/deploy/Dockerfile.detector
~/agrohab/deploy/docker-compose.yml
```

Контейнеры:

```text
agrohab-app       основной сервер на :8000
agrohab-detector  YOLO detector на :8001
```

Проблема была такая:

```text
agrohab-app был запущен с DETECT=0
```

То есть основной сервер принимал кадры, но не запускал YOLO-детекцию, поэтому `/context` оставался пустой:

```json
{
  "frame_seq": 0,
  "objects": [],
  "_stats": {
    "inferences": 0,
    "detector": "http://localhost:8001"
  }
}
```

Еще важный момент:

`localhost:8001` внутри контейнера `agrohab-app` - это не порт хоста, а сам контейнер `agrohab-app`.

Поэтому правильнее запускать app с:

```bash
DETECTOR_URL=http://agrohab-detector:8000
```

и оба контейнера должны быть в одной docker-сети.

## Быстрая проверка состояния

На сервере:

```bash
cd ~/agrohab

printf '123\n' | sudo -S docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}'
```

Проверить основной сервер:

```bash
curl -s http://127.0.0.1:8000/health
echo
```

Проверить context:

```bash
curl -s http://127.0.0.1:8000/context
echo
```

Проверить YOLO detector:

```bash
curl -s http://127.0.0.1:8001/health
echo
```

Хороший ответ YOLO detector:

```json
{
  "ok": true,
  "box_weights": "/weights/best.pt",
  "coco_weights": "/weights/yolo11s.pt",
  "device": "cuda:0"
}
```

## Как поднять только YOLO detector

Если `agrohab-detector` не запущен:

```bash
cd ~/agrohab

printf '123\n' | sudo -S docker rm -f agrohab-detector 2>/dev/null || true

printf '123\n' | sudo -S docker run -d \
  --restart unless-stopped \
  --gpus all \
  --name agrohab-detector \
  -p 8001:8000 \
  -v /home/unitree/agrohab/deploy/weights:/weights:ro \
  -e BOX_WEIGHTS=/weights/best.pt \
  -e COCO_WEIGHTS=/weights/yolo11s.pt \
  -e DETECT_DEVICE=cuda:0 \
  -e DETECT_CONF=0.25 \
  deploy-detector
```

Проверка:

```bash
curl -s http://127.0.0.1:8001/health
echo
```

Логи:

```bash
printf '123\n' | sudo -S docker logs --tail=80 agrohab-detector
```

## Как правильно перезапустить основной сервер app

Нужно, чтобы `agrohab-app` был запущен с:

```bash
DETECT=1
DETECTOR_URL=http://agrohab-detector:8000
SOURCE=robot
```

Сначала подключаем detector к docker-сети app:

```bash
cd ~/agrohab

printf '123\n' | sudo -S docker network connect \
  --alias agrohab-detector \
  deploy_default \
  agrohab-detector 2>/dev/null || true
```

Потом пересоздаем `agrohab-app`.

Важно: эта команда останавливает старый `agrohab-app` и запускает новый.

```bash
cd ~/agrohab

printf '123\n' | sudo -S docker rm -f agrohab-app

printf '123\n' | sudo -S docker run -d \
  --restart unless-stopped \
  --name agrohab-app \
  --network deploy_default \
  -p 8000:8000 \
  -v /home/unitree/agrohab/models:/models:ro \
  -e SOURCE=robot \
  -e PORT=8000 \
  -e DETECT=1 \
  -e DETECTOR_URL=http://agrohab-detector:8000 \
  -e DETECT_EVERY=1 \
  -e BACKEND=vllm \
  -e VLLM_URL=http://vllm:8000/v1 \
  -e MODEL_ID=Qwen/Qwen2.5-VL-7B-Instruct-AWQ \
  -e CAPTION_EVERY=2.0 \
  -e VALIDATE=0 \
  -e GROUNDING=0 \
  -e TARGETS=person,box,cup \
  -e KUZMICH_SYSTEM_PROMPT=/app/kuzmich_system_prompt.txt \
  -e WHISPER_MODEL_PATH=/models/whisper_small \
  -e VOICE_STT_DEVICE=cpu \
  -e VOICE_STT_COMPUTE_TYPE=int8 \
  -e VOICE_TTS_ENGINE=espeak \
  -e VOICE_LANG=ru \
  deploy-app
```

Проверка env после запуска:

```bash
printf '123\n' | sudo -S docker inspect agrohab-app \
  --format '{{range .Config.Env}}{{println .}}{{end}}' \
  | grep -E 'DETECT=|DETECTOR_URL|SOURCE=|PORT='
```

Должно быть:

```text
SOURCE=robot
PORT=8000
DETECT=1
DETECTOR_URL=http://agrohab-detector:8000
```

## Проверка после перезапуска

Проверить контейнеры:

```bash
printf '123\n' | sudo -S docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Ports}}\t{{.Status}}'
```

Должно быть:

```text
agrohab-app       deploy-app       0.0.0.0:8000->8000/tcp
agrohab-detector  deploy-detector  0.0.0.0:8001->8000/tcp
```

Проверить `/health`:

```bash
curl -s http://127.0.0.1:8000/health
echo
```

Проверить `/context`:

```bash
curl -s http://127.0.0.1:8000/context
echo
```

До отправки новых кадров от робота `context` может быть старым. Это нормально.

После запуска робота и отправки кадров `frame_seq` должен стать больше нуля:

```json
{
  "frame_seq": 28,
  "objects": [...]
}
```

Если `frame_seq` остается `0`, значит app все еще не получает или не обрабатывает кадры.

## Проверка с робота

На роботе:

```bash
cd ~/ARMS
EXECUTE=0 MAX_ERROR=0.045 ./run_box_detect_and_plan_fixed_dds.sh
```

В хорошем случае в логе будет:

```text
SERVER_BOX frame=... class=box conf=... X=... Y=... Z=...
BOX_TASK8 detect finished returncode=0
```

В плохом случае:

```text
SERVER_BOX stale/foreign context expected_seq=... last_seq=0
```

Тогда снова проверять сервер:

```bash
curl -s http://127.0.0.1:8000/context
curl -s http://127.0.0.1:8001/health
```

## Почему `frame_match=exact` важен

Робот отправляет кадр на сервер и ждет ответ именно на этот кадр.

Если разрешить брать любой старый `/context`, робот может получить bbox от другого кадра, например от ноутбука или старой камеры.

Поэтому в коде используется:

```bash
CONTEXT_FRAME_MATCH=exact
```

Это правильно. Ошибка была не в `exact`, а в том, что сервер не обновлял `context`.

## Короткая версия: что сделать

На сервере:

```bash
ssh unitree@192.168.10.179
# пароль 123
```

Потом:

```bash
cd ~/agrohab

printf '123\n' | sudo -S docker run -d \
  --restart unless-stopped \
  --gpus all \
  --name agrohab-detector \
  -p 8001:8000 \
  -v /home/unitree/agrohab/deploy/weights:/weights:ro \
  -e BOX_WEIGHTS=/weights/best.pt \
  -e COCO_WEIGHTS=/weights/yolo11s.pt \
  -e DETECT_DEVICE=cuda:0 \
  -e DETECT_CONF=0.25 \
  deploy-detector
```

Если контейнер уже существует, сначала:

```bash
printf '123\n' | sudo -S docker rm -f agrohab-detector
```

Потом включить detector в app:

```bash
printf '123\n' | sudo -S docker network connect \
  --alias agrohab-detector \
  deploy_default \
  agrohab-detector 2>/dev/null || true
```

И перезапустить app с `DETECT=1`, как в большом блоке выше.

## Что уже было сделано во время диагностики

Я проверил:

- `192.168.10.179:8000` отвечал;
- `/context` был пустой: `frame_seq=0`, `objects=[]`;
- `agrohab-app` был запущен с `DETECT=0`;
- `agrohab-detector` сначала не был запущен;
- системный Python не подходит для YOLO: там нет `fastapi`, `uvicorn`, `ultralytics`, `torch`;
- Docker-образ `deploy-detector` уже есть;
- веса `best.pt` и `yolo11s.pt` уже есть;
- `agrohab-detector` удалось поднять на `8001`;
- `/health` детектора ответил `ok=true`.

Оставшаяся обязательная часть: перезапустить `agrohab-app` с `DETECT=1` и `DETECTOR_URL=http://agrohab-detector:8000`.
