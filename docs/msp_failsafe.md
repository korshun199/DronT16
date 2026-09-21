# Удержание после потери связи по MSP

Модуль удержания находится в `src/control/failsafe.py`, а канал датчиков — в
`src/protocols/betaflight_msp_link.py`.

## Схема

```text
FC UART2 TX / PA2 -> Raspberry pin 28 / GPIO1 / UART1 RX
FC UART2 RX / PA3 <- Raspberry pin 27 / GPIO0 / UART1 TX
FC GND            -> Raspberry pin 30 / GND
```

На Raspberry используется `/dev/ttyAMA1`; текущий CRSF остаётся на
`/dev/ttyAMA0`, pin 8/10. Raspberry передаёт Betaflight только RC/setpoint
кадры. PWM моторов и PID остаются внутри FC.

## Алгоритм

В режиме `LIVE` кадры пилота проходят без изменений. Источник потери связи
задаётся `failsafe.mode`: `CH7` использует тумблер, `real` — тайм-аут отсутствия
входных CRSF-кадров. После события потери RPI фиксирует высоту, удерживает её по
барометру и корректирует крен/тангаж к горизонту. После
подтверждения горизонтального положения выполняется разворот на 180 градусов
командой yaw до подтверждения заданного угла датчиком yaw. Ограничения по времени
нет. Затем yaw центрируется, высота и горизонт удерживаются без ограничения
времени до CH7=LINK_OK или DISARM.

Состояния: `LIVE -> LEVELING -> TURNING -> HOLDING`. При пропаже свежих MSP
данных используется `FAULT` согласно `fault_action`; DISARM имеет высший приоритет.
В режиме `real` новые команды пилота во время радиопотери физически недоступны;
возврат в `LIVE` происходит после появления новых корректных RC-кадров.
Состояний `DESCENT`, `LANDED` и автоматической посадки в проекте больше нет.

## Настройки

```toml
[msp]
enabled = true

[failsafe]
enabled = true
mode = "CH7"
turn_degrees = 180.0
turn_yaw_command = 1155
turn_duration_s = 3.5
```

Все параметры failsafe находятся в `config/bridge.toml` и снабжены русскими
комментариями. Разворот выполняется ровно `turn_duration_s` секунд; показание yaw
записывается в журнал, но не завершает разворот досрочно.

## Проверки

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile src/protocols/betaflight_msp_link.py src/control/failsafe.py scripts/crsf_bridge.py
```

До стендовой проверки без винтов нужно подтвердить в журнале свежие MSP-пакеты:

```text
[FC] SENSOR: altitude=... vario=... roll=... pitch=... yaw=...
```
