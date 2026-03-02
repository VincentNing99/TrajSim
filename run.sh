#!/bin/zsh
set -e
cd "$(dirname "$0")"

# ── helpers ──────────────────────────────────────────────────────────────────
usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  test           Run tests instead of simulation"
    echo "  --clean        Remove build directory and reconfigure"
    echo "  --mission N    Use config/config_mission_N.json (default: 1)"
    echo "  --help         Show this help"
    exit 0
}

BUILD_DIR="build"
MISSION=1
RUN_TESTS=false
CLEAN=false

# ── arg parsing ───────────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        test)        RUN_TESTS=true ;;
        --clean)     CLEAN=true ;;
        --mission=*) MISSION="${arg#--mission=}" ;;
        --mission)   ;;  # handled below with shift-like logic
        --help|-h)   usage ;;
    esac
done
# handle "--mission N" (two-arg form)
for i in {1..$#}; do
    if [[ "${@[$i]}" == "--mission" && -n "${@[$((i+1))]}" ]]; then
        MISSION="${@[$((i+1))]}"
    fi
done

CONFIG="config/config_mission_${MISSION}.json"

# ── clean ─────────────────────────────────────────────────────────────────────
if [[ "$CLEAN" == true ]]; then
    echo "==> Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# ── configure ─────────────────────────────────────────────────────────────────
if [[ ! -d "$BUILD_DIR" ]]; then
    echo "==> Configuring CMake..."
    cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
fi

# ── build ─────────────────────────────────────────────────────────────────────
echo "==> Building..."
cmake --build "$BUILD_DIR" --config Release 2>&1 \
    | grep -E "error:|warning:|Building|Linking|Built target" \
    | grep -vE "warning:.*(nodiscard|unused)"

# ── run ───────────────────────────────────────────────────────────────────────
if [[ "$RUN_TESTS" == true ]]; then
    echo "==> Running tests..."
    ./"$BUILD_DIR"/trajsim_tests
else
    if [[ ! -f "$CONFIG" ]]; then
        echo "Error: config file '$CONFIG' not found" >&2
        exit 1
    fi
    echo "==> Running simulation (mission ${MISSION})..."
    # Pass mission config via env var or sed-patch main if needed
    # Currently main.cpp hardcodes config_mission_1.json, so warn if different
    if [[ "$MISSION" != "1" ]]; then
        echo "[WARN] main.cpp hardcodes config_mission_1.json — mission ${MISSION} config not auto-applied"
    fi
    ./"$BUILD_DIR"/trajsim
fi
