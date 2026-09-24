#!/usr/bin/env python3
"""Анализатор CRSF-кадров из сохранённой сессии PulseView (.sr)."""

from __future__ import annotations

import argparse
import re
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path


# Адрес начала стандартного CRSF-кадра от приёмника.
CRSF_ADDRESS_RECEIVER = 0xC8
# Тип упакованного кадра шестнадцати RC-каналов.
CRSF_RC_CHANNELS_PACKED = 0x16
# Порог канала CH5, выше которого передатчик считается в ARM.
DEFAULT_ARM_THRESHOLD = 1700
# Скорость CRSF по умолчанию для проекта DronT16.
DEFAULT_BAUDRATE = 420_000
# Номер входа D6 в PulseView: D0 имеет номер 0.
DEFAULT_CHANNEL = 6


@dataclass(frozen=True)
class CaptureData:
    """Исходные отсчёты цифрового канала и частота захвата."""

    samples: bytes
    sample_rate: int


@dataclass(frozen=True)
class AnalysisResult:
    """Результаты разбора одной сессии PulseView."""

    decoded_bytes: int
    valid_frames: tuple[tuple[int, ...], ...]
    bad_crc_frames: int
    candidate_frames: int


def read_capture(path: Path) -> CaptureData:
    """Читает цифровые отсчёты и samplerate из архива PulseView."""

    if not path.is_file():
        raise FileNotFoundError(f"Файл сессии не найден: {path}")

    try:
        with zipfile.ZipFile(path) as archive:
            metadata = archive.read("metadata").decode("utf-8", errors="replace")
            sample_rate = parse_sample_rate(metadata)
            chunk_names = sorted(
                (name for name in archive.namelist() if name.startswith("logic-1-")),
                key=logic_chunk_number,
            )
            if not chunk_names:
                raise ValueError("В сессии нет цифровых отсчётов logic-1-*")
            samples = b"".join(archive.read(name) for name in chunk_names)
    except zipfile.BadZipFile as error:
        raise ValueError(f"Файл не является корректной сессией PulseView: {path}") from error
    except KeyError as error:
        raise ValueError(f"В сессии PulseView отсутствует файл: {error.args[0]}") from error

    return CaptureData(samples=samples, sample_rate=sample_rate)


def parse_sample_rate(metadata: str) -> int:
    """Извлекает частоту захвата из metadata PulseView."""

    match = re.search(r"^samplerate=(\d+)\s*MHz\s*$", metadata, flags=re.MULTILINE)
    if match is None:
        raise ValueError("В metadata не найдена samplerate")
    return int(match.group(1)) * 1_000_000


def logic_chunk_number(name: str) -> int:
    """Возвращает номер части logic-1-N для правильного порядка чтения."""

    match = re.search(r"logic-1-(\d+)$", name)
    return int(match.group(1)) if match else -1


def crc8_dvb_s2(data: bytes) -> int:
    """Вычисляет CRC8-DVB-S2, используемую в CRSF."""

    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def decode_uart(samples: bytes, sample_rate: int, baudrate: int, channel: int) -> bytes:
    """Декодирует выбранный логический канал как UART 8N1, LSB first."""

    if not 0 <= channel <= 7:
        raise ValueError("Номер канала PulseView должен быть от 0 до 7")
    if baudrate <= 0 or sample_rate <= baudrate:
        raise ValueError("Скорость UART и частота захвата заданы неверно")

    samples_per_bit = sample_rate / baudrate
    signal = ((sample >> channel) & 1 for sample in samples)
    levels = list(signal)
    decoded = bytearray()
    index = 1
    last_start = len(levels) - int(10 * samples_per_bit)

    while index < last_start:
        # UART в покое находится в единице; переход 1->0 — начало кадра.
        if levels[index - 1] != 1 or levels[index] != 0:
            index += 1
            continue

        start_sample = int(index + 0.5 * samples_per_bit)
        if levels[start_sample] != 0:
            index += 1
            continue

        value = 0
        valid = True
        for bit in range(8):
            sample_index = int(index + (1.5 + bit) * samples_per_bit)
            if sample_index >= len(levels):
                valid = False
                break
            value |= levels[sample_index] << bit

        stop_index = int(index + 9.5 * samples_per_bit)
        if valid and stop_index < len(levels) and levels[stop_index] == 1:
            decoded.append(value)
            # Пропускаем обработанный байт; следующий байт ищется с его конца.
            index = int(index + 10 * samples_per_bit)
        else:
            index += 1

    return bytes(decoded)


def unpack_channels(payload: bytes) -> tuple[int, ...]:
    """Распаковывает шестнадцать 11-битных RC-каналов CRSF."""

    if len(payload) != 22:
        raise ValueError(f"Ожидалось 22 байта каналов, получено {len(payload)}")
    packed = int.from_bytes(payload, byteorder="little")
    return tuple((packed >> (11 * channel)) & 0x7FF for channel in range(16))


def analyze_bytes(data: bytes) -> AnalysisResult:
    """Находит RC_CHANNELS_PACKED и проверяет CRC каждого кадра."""

    valid_frames: list[tuple[int, ...]] = []
    candidate_frames = 0
    bad_crc_frames = 0
    index = 0

    while index + 2 < len(data):
        if data[index] != CRSF_ADDRESS_RECEIVER:
            index += 1
            continue
        frame_length = data[index + 1]
        frame_end = index + 2 + frame_length
        if frame_length < 2 or frame_end > len(data):
            index += 1
            continue
        if data[index + 2] != CRSF_RC_CHANNELS_PACKED:
            index += 1
            continue

        candidate_frames += 1
        frame = data[index:frame_end]
        if len(frame) == 26 and crc8_dvb_s2(frame[2:-1]) == frame[-1]:
            valid_frames.append(unpack_channels(frame[3:-1]))
        else:
            bad_crc_frames += 1
        index += frame_length + 2

    return AnalysisResult(
        decoded_bytes=len(data),
        valid_frames=tuple(valid_frames),
        bad_crc_frames=bad_crc_frames,
        candidate_frames=candidate_frames,
    )


def color(text: str, code: str, enabled: bool) -> str:
    """Добавляет цвет ANSI, если вывод выполняется в цветном режиме."""

    return f"\033[{code}m{text}\033[0m" if enabled else text


def state_sequence(result: AnalysisResult, arm_threshold: int) -> tuple[str, ...]:
    """Возвращает последовательность изменений состояния CH5."""

    states: list[str] = []
    for channels in result.valid_frames:
        state = "ARM" if channels[4] >= arm_threshold else "DISARM"
        if not states or states[-1] != state:
            states.append(state)
    return tuple(states)


def print_channel_report(
    role: str,
    result: AnalysisResult,
    sample_rate: int,
    channel: int,
    baudrate: int,
    arm_threshold: int,
    show_channels: bool,
    use_color: bool,
) -> None:
    """Печатает отчёт одного направления CRSF."""

    print(f"[{role}] UART: D{channel}, {baudrate} бод, 8N1; захват: {sample_rate / 1_000_000:g} MHz")
    print(f"UART-байтов: {result.decoded_bytes}")
    print(
        f"Кандидатов RC: {result.candidate_frames}; "
        f"валидных: {len(result.valid_frames)}; CRC ошибок: {result.bad_crc_frames}"
    )

    if not result.valid_frames:
        print(color("ОШИБКА: валидные CRSF RC-кадры не найдены", "31", use_color))
        return

    previous_state: str | None = None
    for frame_number, channels in enumerate(result.valid_frames, start=1):
        ch5 = channels[4]
        state = "ARM" if ch5 >= arm_threshold else "DISARM"
        if state == previous_state:
            continue
        previous_state = state
        state_color = "32" if state == "ARM" else "33"
        print(
            color(
                f"{state}: кадр={frame_number} CH5={ch5} "
                f"(порог ARM >= {arm_threshold})",
                state_color,
                use_color,
            )
        )
        if show_channels:
            print("  " + " ".join(f"CH{i}={value}" for i, value in enumerate(channels, start=1)))


def print_comparison(
    input_result: AnalysisResult,
    output_result: AnalysisResult,
    arm_threshold: int,
    use_color: bool,
) -> None:
    """Сравнивает состояния и каналы на входе и выходе моста."""

    input_states = state_sequence(input_result, arm_threshold)
    output_states = state_sequence(output_result, arm_threshold)
    input_has_arm = "ARM" in input_states
    output_has_arm = "ARM" in output_states
    input_has_disarm = "DISARM" in input_states
    output_has_disarm = "DISARM" in output_states

    print(f"Входные состояния:  {' -> '.join(input_states) or 'нет данных'}")
    print(f"Выходные состояния: {' -> '.join(output_states) or 'нет данных'}")
    if input_states == output_states and input_has_arm and input_has_disarm:
        message = "OK: ARM и DISARM присутствуют на входе и выходе моста"
        code = "32"
    elif input_states == output_states and input_has_arm:
        message = "OK: ARM передан на выход; DISARM в этой записи не захватывался"
        code = "32"
    elif input_states == output_states and input_has_disarm:
        message = "OK: DISARM передан на выход; ARM в этой записи не захватывался"
        code = "32"
    elif input_has_arm and not output_has_arm:
        message = "ОШИБКА: ARM есть на входе, но отсутствует на выходе моста"
        code = "31"
    elif not input_has_arm:
        message = "ПРЕДУПРЕЖДЕНИЕ: на входе нет ARM; выход сравнить невозможно"
        code = "33"
    else:
        message = "ПРЕДУПРЕЖДЕНИЕ: требуется проверить полноту записи и состояния CH5"
        code = "33"
    print(color(message, code, use_color))


def analyze_channel(capture: CaptureData, baudrate: int, channel: int) -> AnalysisResult:
    """Декодирует и анализирует один канал PulseView."""

    uart_data = decode_uart(capture.samples, capture.sample_rate, baudrate, channel)
    return analyze_bytes(uart_data)


def build_parser() -> argparse.ArgumentParser:
    """Создаёт интерфейс командной строки анализатора."""

    parser = argparse.ArgumentParser(description="Сравнение входящего и исходящего CRSF из PulseView .sr")
    parser.add_argument("capture", type=Path, help="путь к файлу PulseView .sr")
    parser.add_argument(
        "--input-channel",
        type=int,
        default=DEFAULT_CHANNEL,
        help="канал входа приёмника в Raspberry; D6 = 6 по умолчанию",
    )
    parser.add_argument(
        "--output-channel",
        type=int,
        required=True,
        help="канал выхода Raspberry к полётнику; например D5 = 5",
    )
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="скорость CRSF")
    parser.add_argument("--arm-threshold", type=int, default=DEFAULT_ARM_THRESHOLD, help="порог ARM для CH5")
    parser.add_argument("--channels", action="store_true", help="печатать CH1..CH16 при смене состояния")
    parser.add_argument("--no-color", action="store_true", help="отключить цветной вывод")
    return parser


def main() -> int:
    """Запускает чтение сессии, декодирование и вывод отчёта."""

    parser = build_parser()
    arguments = parser.parse_args()
    try:
        capture = read_capture(arguments.capture)
        input_result = analyze_channel(capture, arguments.baudrate, arguments.input_channel)
        output_result = analyze_channel(capture, arguments.baudrate, arguments.output_channel)
        print(f"Файл: {arguments.capture}")
        print_channel_report(
            "INPUT RX приёмника",
            input_result,
            capture.sample_rate,
            arguments.input_channel,
            arguments.baudrate,
            arguments.arm_threshold,
            arguments.channels,
            use_color=not arguments.no_color and sys.stdout.isatty(),
        )
        print()
        print_channel_report(
            "OUTPUT TX Raspberry",
            output_result,
            capture.sample_rate,
            arguments.output_channel,
            arguments.baudrate,
            arguments.arm_threshold,
            arguments.channels,
            use_color=not arguments.no_color and sys.stdout.isatty(),
        )
        print()
        print_comparison(
            input_result,
            output_result,
            arguments.arm_threshold,
            use_color=not arguments.no_color and sys.stdout.isatty(),
        )
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        print(f"ОШИБКА: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
