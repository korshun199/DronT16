# Скрипты DronT16

В каталоге находятся только рабочие вспомогательные файлы текущей конфигурации.

## Рабочие входы

- `../run.sh` — единственная команда запуска всей системы;
- `../run.sh sensor-test` — локальный тест привязки MSP-датчиков без UART,
  камеры, CRSF и команд полётнику;
- `crsf_bridge.py` — управляемый CRSF/MSP-мост, запускаемый из `run.sh`;
- `rpi.sh` — проверка Raspberry, чтение журнала и безопасная синхронизация;

Разовые стендовые диагностики и старый текстовый симулятор удалены из рабочей
копии. Их исходный код остаётся доступен в истории Git и в локальной резервной
копии Raspberry, но не входит в текущий запуск и не синхронизируется на плату.

`sensor_binding_test.py` прогоняет синтетические MSPv1-ответы через настоящий
`MspParser`, проверяет CRC, свежесть, единицы и объединение показаний барометра,
IMU, магнитометра/сырого диагностического курса и GPS. Результат пишется в
`diagnostics/raspberry/sensor_binding_test.log`.

## Правило конфигурации

Скрипты не создают собственные TOML-файлы. Python-код получает значения через
`src/configuration.py`, а каждый модуль читает только свою секцию:

```text
config/dront16.toml
        ├── video, osd.*
        ├── follow.*
        ├── receiver.*
        ├── bridge, msp, failsafe, visual_servoing
        ├── simulation.*
        └── logging, runtime
```
# Скрипты DronT16

## Анализ CRSF-записи PulseView

`analyze_crsf_capture.py` разбирает сохранённую сессию PulseView `.sr` без
подключения к Raspberry. Он отдельно декодирует вход CRSF от приёмника и
выход CRSF Raspberry, проверяет CRC, распаковывает CH1–CH16 и сравнивает
переходы CH5 между `ARM` и `DISARM`.

Запуск для входа на D6 и выхода на D5:

```bash
python3 scripts/analyze_crsf_capture.py diagnostics/pulseview/test_dv.sr --input-channel 6 --output-channel 5
```

Полный снимок каналов при каждом изменении CH5:

```bash
python3 scripts/analyze_crsf_capture.py diagnostics/pulseview/test_dv.sr --input-channel 6 --output-channel 5 --channels
```

Если анализатор подключён к другим входам PulseView, номера можно изменить.
Например, вход на D6 и выход на D7:

```bash
python3 scripts/analyze_crsf_capture.py diagnostics/pulseview/test_dv.sr --input-channel 6 --output-channel 7
```
