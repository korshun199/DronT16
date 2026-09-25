#!/usr/bin/env bash
# Единый рабочий инструмент DronT16 для Raspberry Pi.
# Не меняет прошивки, GPIO, systemd или сеть. Синхронизация исходников
# выполняется только по явной команде sync --apply.

set -Eeuo pipefail

# Корень локального проекта определяется относительно этого скрипта.
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Карта конкретной Raspberry Pi локальна и не публикуется в Git.
CONNECTION_CONFIG="${DRONT16_CONNECTION_CONFIG:-${PROJECT_DIR}/config/deployment.toml}"
# Короткий тайм-аут не даёт зависнуть при отключённой Raspberry Pi.
CONNECT_TIMEOUT="${DRONT16_RPI_TIMEOUT:-8}"

fail() {
    # Печатает понятную ошибку и завершает операцию.
    printf '[RPI] ОШИБКА: %s\n' "$1" >&2
    exit 1
}

config_value() {
    # Читает простое значение из TOML без выполнения его как shell-кода.
    local section="$1"
    local key="$2"
    awk -F '=' -v wanted="$key" -v wanted_section="$section" '
        /^\[[^]]+\]/ { current = $0; gsub(/^\[|\]$/, "", current); next }
        current != wanted_section { next }
        $1 ~ "^[[:space:]]*" wanted "[[:space:]]*$" {
            value = $2
            sub(/^[[:space:]]*/, "", value)
            sub(/[[:space:]]*#.*/, "", value)
            gsub(/^"|"$/, "", value)
            print value
            exit
        }
    ' "$CONNECTION_CONFIG"
}

load_connection() {
    # Загружает один источник адреса, пути, службы и журнала.
    [[ -r "$CONNECTION_CONFIG" ]] || fail "нет $CONNECTION_CONFIG; создай его из config/deployment.example.toml"
    RPI_HOST="${DRONT16_RPI_HOST:-$(config_value raspberry host)}"
    RPI_USER="${DRONT16_RPI_USER:-$(config_value raspberry user)}"
    RPI_PROJECT_DIR="$(config_value raspberry project_path)"
    RPI_SERVICE="$(config_value raspberry service)"
    RPI_LOG_PATH="$(config_value raspberry log_path)"
    RPI_LOG_DIR="$(dirname "$RPI_LOG_PATH")"
    RPI_LOG_STEM="$(basename "${RPI_LOG_PATH%.log}")"
    local key_name
    key_name="$(basename "$(config_value raspberry ssh_key)")"
    RPI_KEY="${DRONT16_SSH_KEY:-${HOME}/.ssh/${key_name}}"
    [[ -n "$RPI_HOST" && -n "$RPI_USER" && -n "$RPI_PROJECT_DIR" ]] || fail 'неполная секция [raspberry] в deployment.toml'
    [[ -r "$RPI_KEY" ]] || fail "SSH-ключ не найден: $RPI_KEY"
    command -v ssh >/dev/null 2>&1 || fail 'ssh не установлен на ноутбуке'
}

remote_latest_log() {
    # Возвращает последний завершённый журнал ARM-цикла с Raspberry Pi.
    local command
    command="latest=\$(find '${RPI_LOG_DIR}' -maxdepth 1 -type f -name '${RPI_LOG_STEM}_*.log' -printf '%T@ %p\\n' | sort -n | tail -n 1 | cut -d' ' -f2-); if [ -z \"\$latest\" ] && [ -f '${RPI_LOG_PATH}' ]; then latest='${RPI_LOG_PATH}'; fi; if [ -z \"\$latest\" ]; then echo '[RPI] журнал не найден' >&2; exit 1; fi; printf '%s' \"\$latest\""
    ssh_run "$command"
}

ssh_run() {
    # Выполняет команду на точно определённой Raspberry Pi.
    ssh -i "$RPI_KEY" -o IdentitiesOnly=yes -o BatchMode=yes \
        -o "ConnectTimeout=${CONNECT_TIMEOUT}" -o ServerAliveInterval=15 \
        -o ServerAliveCountMax=3 "${RPI_USER}@${RPI_HOST}" "$@"
}

show_config() {
    # Показывает карту подключения без обращения к сети.
    printf '[RPI] host=%s user=%s\n' "$RPI_HOST" "$RPI_USER"
    printf '[RPI] project=%s service=%s\n' "$RPI_PROJECT_DIR" "$RPI_SERVICE"
    printf '[RPI] log=%s key=%s\n' "$RPI_LOG_PATH" "$RPI_KEY"
}

check_remote() {
    # Проверяет связь, время, службу, процессы и доступность журнала.
    local command remote_log
    command="printf '[RPI] hostname='; hostname; printf '[RPI] addresses='; hostname -I; date '+[RPI] time=%F %T.%3N %Z %z'; timedatectl show --property=Timezone --property=NTPSynchronized --property=NTP --value 2>/dev/null | sed 's/^/[RPI] time-status=/'; printf '[RPI] service='; systemctl is-active '${RPI_SERVICE}' 2>/dev/null || true; printf '[RPI] processes:\\n'; pgrep -af 'crsf_bridge.py|src.app|(^|/)run\\.sh( |$)' || true"
    ssh_run "$command"
    if remote_log="$(remote_latest_log 2>/dev/null)"; then
        ssh_run "printf '[RPI] last-log='; basename '${remote_log}'; printf '[RPI] log-lines='; wc -l < '${remote_log}'"
    else
        printf '[RPI] last-log=missing\n[RPI] log-lines=missing\n'
    fi
}

show_log_tail() {
    # Показывает хвост журнала; нулевые байты удаляются только из вывода.
    local lines="${1:-120}" remote_log
    [[ "$lines" =~ ^[0-9]+$ ]] || fail 'число строк журнала должно быть целым'
    remote_log="$(remote_latest_log)"
    ssh_run "tr -d '\\000' < '${remote_log}' | tail -n ${lines}"
}

show_last_cycle() {
    # Извлекает последний ARM -> DISARM цикл без ручного поиска в полном логе.
    local command remote_log
    remote_log="$(remote_latest_log)"
    command="tr -d '\\000' < '${remote_log}' | awk '/\\[PILOT\\] ARM SWITCH: .* -> ARM/ { current = \$0 ORS; active = 1; next } active { current = current \$0 ORS } active && /\\[PILOT\\] ARM SWITCH: .* -> DISARM/ { last = current; active = 0 } END { if (active) last = current; if (last != \"\") printf \"%s\", last; else exit 2 }'"
    ssh_run "$command"
}

safe_file_list() {
    # Перечисляет только воспроизводимые файлы проекта для передачи на Raspberry.
    # Передаём только отслеживаемые Git-файлы. Неотслеживаемые записи PulseView,
    # видео, логи и временные каталоги никогда не должны уходить на устройство.
    git -C "$PROJECT_DIR" ls-files | while IFS= read -r relative_path; do
        [[ -f "$PROJECT_DIR/$relative_path" ]] || continue
        case "$relative_path" in
            config/deployment.toml|security/*|security_sample/*|backups/*|docs/BTFL_cli_*.txt|*.log) continue ;;
        esac
        printf '%s\n' "$relative_path"
    done
}

sync_project() {
    # По умолчанию показывает preview; --apply действительно записывает файлы.
    local apply="${1:-}"
    [[ -z "$apply" || "$apply" == "--apply" ]] || fail 'используй sync или sync --apply'
    command -v rsync >/dev/null 2>&1 || fail 'rsync не установлен на ноутбуке'
    # rsync использует тот же ключ и ограничения, что и все SSH-проверки.
    local remote_shell
    remote_shell="ssh -i ${RPI_KEY} -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=${CONNECT_TIMEOUT}"
    local -a options=(-az --human-readable --itemize-changes --files-from=- -e "$remote_shell")
    if [[ "$apply" != "--apply" ]]; then
        options+=(-n)
        printf '[RPI] SYNCHRONIZE PREVIEW: Raspberry не изменяется\n'
    else
        printf '[RPI] SYNCHRONIZE APPLY: служба не перезапускается\n'
    fi
    safe_file_list | rsync "${options[@]}" "$PROJECT_DIR/" "${RPI_USER}@${RPI_HOST}:${RPI_PROJECT_DIR}/"
}

copy_log() {
    # Сохраняет последний журнал только в diagnostics/raspberry. Старые копии
    # не перезаписываются, чтобы анализ оставался воспроизводимым.
    local output_dir output_file remote_log remote_name
    remote_log="$(remote_latest_log)"
    remote_name="$(basename "$remote_log")"
    output_dir="$PROJECT_DIR/diagnostics/raspberry"
    output_file="$output_dir/$remote_name"
    mkdir -p "$output_dir"
    if [[ -e "$output_file" ]]; then
        printf '[RPI] Этот журнал уже сохранён: %s\n' "$output_file"
        return 0
    fi
    ssh_run "cat '${remote_log}'" > "$output_file"
    printf '[RPI] Лог сохранён локально: %s\n' "$output_file"
}

verify_remote() {
    # Проверяет загруженный код в окружении Raspberry Pi без перезапуска службы.
    local command
    command="cd '${RPI_PROJECT_DIR}' && .venv/bin/python3 -m compileall -q src scripts && .venv/bin/python3 -m unittest discover -s tests -q && echo '[RPI] VERIFY: syntax and tests OK'"
    ssh_run "$command"
}

show_help() {
    # Печатает короткую справку по эксплуатации Raspberry Pi.
    cat <<'EOF'
Использование:
  ./scripts/rpi.sh check             связь, часы, служба, процессы и журнал
  ./scripts/rpi.sh log [строки]      хвост рабочего журнала Raspberry Pi
  ./scripts/rpi.sh cycle             последний ARM -> DISARM цикл журнала
  ./scripts/rpi.sh sync              preview синхронизации без записи
  ./scripts/rpi.sh sync --apply      загрузить безопасные файлы без рестарта
  ./scripts/rpi.sh verify            проверить синтаксис и тесты на Raspberry
  ./scripts/rpi.sh copy-log          сохранить последний журнал в diagnostics/raspberry/
  ./scripts/rpi.sh shell             открыть SSH-оболочку
  ./scripts/rpi.sh config            показать карту подключения

После проверенной загрузки службу перезапускает владелец:
  sudo systemctl restart dront16
EOF
}

main() {
    # Выполняет только фиксированный набор команд, чтобы не путать стенды.
    load_connection
    case "${1:-check}" in
        check) check_remote ;;
        log) show_log_tail "${2:-120}" ;;
        cycle) show_last_cycle ;;
        sync) sync_project "${2:-}" ;;
        verify) verify_remote ;;
        copy-log) copy_log ;;
        shell) ssh_run ;;
        config) show_config ;;
        help|--help|-h) show_help ;;
        *) fail "неизвестная команда: ${1}; используй ./scripts/rpi.sh help" ;;
    esac
}

main "$@"
