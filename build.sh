#!/usr/bin/env bash
set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXPORT_SCRIPT="${ESP_IDF_EXPORT_SCRIPT:-${HOME}/esp/esp-idf/export.sh}"

# Default State
PORT="${ESPPORT:-}"
RUN_MONITOR=0
BUILD_ONLY=0
SELECTED_BOARD=""

# 硬件定义：使用 Bash 3.2 兼容的 case，避免 macOS 系统 Bash 不支持关联数组。

usage() {
    cat <<EOF
Usage: $(basename "$0") --board <name> [options]
Boards: esp32s3, m5stack_sticks3, waveshare_c6, waveshare_c6_1.9, waveshare_c6_1.47, t_embed
Options: -p|--port, -m|--monitor, --build-only
EOF
}

detect_port() {
    local matches=()
    shopt -s nullglob
    matches=(/dev/cu.usbmodem* /dev/cu.usbserial* /dev/ttyACM*)
    shopt -u nullglob
    if [[ "${#matches[@]}" -eq 1 ]]; then
        printf '%s\n' "${matches[0]}"
    elif [[ "${#matches[@]}" -gt 1 ]]; then
        echo "Multiple ports found: ${matches[*]}. Specify with --port." >&2
        return 1
    else
        [[ "${BUILD_ONLY}" -eq 0 ]] && echo "No device found." >&2
        return 1
    fi
}

release_port() {
    local port="$1"
    [[ -z "${port}" || ! -e "${port}" ]] && return 0
    if command -v lsof >/dev/null 2>&1; then
        local pids
        pids="$(lsof -t "${port}" 2>/dev/null || true)"
        [[ -n "${pids}" ]] && kill -9 ${pids} && sleep 0.3
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -b|--board)   SELECTED_BOARD="$2"; shift 2 ;;
        -p|--port)    PORT="$2"; shift 2 ;;
        -m|--monitor) RUN_MONITOR=1; shift ;;
        --build-only) BUILD_ONLY=1; shift ;;
        -h|--help)    usage; exit 0 ;;
        *) echo "Unknown: $1"; usage; exit 1 ;;
    esac
done

if [[ -z "${SELECTED_BOARD}" ]]; then
    echo "Error: Valid --board required (esp32s3, m5stack_sticks3, waveshare_c6, t_embed)." >&2
    exit 1
fi

# 将用户选择映射到固件板名、构建目录和 ESP-IDF 目标。
case "${SELECTED_BOARD}" in
    esp32s3)
        BOARD="esp32s3_generic"; BUILD_DIR="build_s3"; TARGET="esp32s3" ;;
    m5stack_sticks3)
        BOARD="m5stack_sticks3"; BUILD_DIR="build_m5stack_sticks3"; TARGET="esp32s3" ;;
    waveshare_c6)
        BOARD="waveshare_c6_1.9"; BUILD_DIR="build_waveshare_c6"; TARGET="esp32c6" ;;
    waveshare_c6_1.9)
        BOARD="waveshare_c6_1.9"; BUILD_DIR="build_waveshare_c6"; TARGET="esp32c6" ;;
    waveshare_c6_1.47)
        BOARD="waveshare_c6_1.47"; BUILD_DIR="build_waveshare_c6_1.47"; TARGET="esp32c6" ;;
    t_embed)
        BOARD="lilygo_t_embed_cc1101"; BUILD_DIR="build_t_embed"; TARGET="esp32s3" ;;
    *)
        echo "Error: Unknown board '${SELECTED_BOARD}'." >&2
        exit 1 ;;
esac

# Force environment to match board target
export IDF_TARGET="${TARGET}"

if [[ -z "${PORT}" && "${BUILD_ONLY}" -eq 0 ]]; then
    PORT="$(detect_port || echo "")"
    [[ -z "${PORT}" ]] && exit 1
fi

[[ ! -f "${EXPORT_SCRIPT}" ]] && echo "IDF export script missing." >&2 && exit 1
# shellcheck source=/dev/null
source "${EXPORT_SCRIPT}"

cd "${SCRIPT_DIR}"

[[ "${BUILD_ONLY}" -eq 0 ]] && release_port "${PORT}"

# Remove root sdkconfig if it belongs to a different target
if [[ -f "sdkconfig" ]]; then
    CURRENT_CONFIG_TARGET=$(sed -n 's/^CONFIG_IDF_TARGET="\([^"]*\)"/\1/p' sdkconfig 2>/dev/null || echo "")
    if [[ -z "${CURRENT_CONFIG_TARGET}" || "${CURRENT_CONFIG_TARGET}" != "${TARGET}" ]]; then
        echo "Root sdkconfig mismatch (Config: '${CURRENT_CONFIG_TARGET}', Needed: '${TARGET}'). Removing..."
        rm -f sdkconfig
    fi
fi

# Remove build-dir sdkconfig if it belongs to a different target
if [[ -f "${BUILD_DIR}/sdkconfig" ]]; then
    BD_TARGET=$(sed -n 's/^CONFIG_IDF_TARGET="\([^"]*\)"/\1/p' "${BUILD_DIR}/sdkconfig" 2>/dev/null || echo "")
    if [[ -z "${BD_TARGET}" || "${BD_TARGET}" != "${TARGET}" ]]; then
        echo "Build dir sdkconfig mismatch (Config: '${BD_TARGET}', Needed: '${TARGET}'). Removing..."
        rm -f "${BUILD_DIR}/sdkconfig"
    fi
fi

# Check for board-specific sdkconfig defaults
BOARD_DEFAULTS_OPTS=()
if [[ -f "sdkconfig.defaults.${BOARD}" ]]; then
    BOARD_DEFAULTS_OPTS=("-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.${BOARD}")
fi

# Set target (creates/updates sdkconfig)
idf.py -B "${BUILD_DIR}" "${BOARD_DEFAULTS_OPTS[@]}" set-target "${TARGET}"

# Construct command
COMMANDS=("reconfigure" "build")
PY_OPTS=("-B" "${BUILD_DIR}" "-DFLIPPER_BOARD=${BOARD}" "${BOARD_DEFAULTS_OPTS[@]}")

if [[ "${BUILD_ONLY}" -eq 0 ]]; then
    COMMANDS+=("flash")
    PY_OPTS+=("-p" "${PORT}")
    [[ "${RUN_MONITOR}" -eq 1 ]] && COMMANDS+=("monitor")
fi

idf.py "${PY_OPTS[@]}" "${COMMANDS[@]}"
