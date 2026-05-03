#!/usr/bin/env bash
# Swap the params file the rover-platform-hal systemd unit launches with.
#
#   switch-platform-config.sh prod   → use deploy/config/platform_hal.yaml
#                                       (production: PCA9685 backend)
#   switch-platform-config.sh bench  → use deploy/config/platform_hal-bench.yaml
#                                       (development: mock GPIO backend)
#   switch-platform-config.sh status → print which YAML is currently wired in
#
# Re-installs the unit fresh from the repo each time (so any edits to the
# repo's systemd file land on the system). Requires sudo and reloads +
# restarts the rover-platform-hal unit.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC_UNIT="$REPO_ROOT/deploy/systemd/rover-platform-hal.service"
DST_UNIT="/etc/systemd/system/rover-platform-hal.service"
PROD_YAML="$REPO_ROOT/deploy/config/platform_hal.yaml"
BENCH_YAML="$REPO_ROOT/deploy/config/platform_hal-bench.yaml"

usage() {
    echo "usage: $(basename "$0") {prod|bench|status}" >&2
    exit 2
}

[[ $# -eq 1 ]] || usage

read_current_params_file() {
    if [[ ! -f "$DST_UNIT" ]]; then
        echo ""
        return
    fi
    # `|| true` so a missing match doesn't trip `pipefail`.
    grep -oE "params_file:=[^ ']+" "$DST_UNIT" | sed 's/params_file:=//' | head -1 || true
}

install_with_params() {
    local target="$1"
    if [[ ! -f "$SRC_UNIT" ]]; then
        echo "error: source unit missing: $SRC_UNIT" >&2
        exit 1
    fi
    if [[ ! -f "$target" ]]; then
        echo "error: target YAML missing: $target" >&2
        exit 1
    fi
    # Copy the repo unit to /etc, then sed the params_file into ExecStart.
    # The repo unit already references the prod YAML; rewriting it lets us
    # point at either prod or bench from one source of truth.
    local escaped
    escaped=$(printf '%s' "$target" | sed 's:[/&]:\\&:g')
    sudo install -m 0644 "$SRC_UNIT" "$DST_UNIT"
    sudo sed -i -E "s|params_file:=[^ ']+|params_file:=${escaped}|" "$DST_UNIT"
    sudo systemctl daemon-reload
    sudo systemctl restart rover-platform-hal.service
}

case "$1" in
    status)
        current=$(read_current_params_file)
        if [[ ! -f "$DST_UNIT" ]]; then
            echo "rover-platform-hal.service is not installed at $DST_UNIT"
        elif [[ -z "$current" ]]; then
            echo "installed unit has no params_file (stale or hand-edited): $DST_UNIT"
        else
            label="other"
            [[ "$current" == "$PROD_YAML" ]]  && label="prod"
            [[ "$current" == "$BENCH_YAML" ]] && label="bench"
            echo "current ($label): $current"
        fi
        ;;
    prod)
        install_with_params "$PROD_YAML"
        echo "switched to prod ($PROD_YAML); unit restarted"
        ;;
    bench)
        install_with_params "$BENCH_YAML"
        echo "switched to bench ($BENCH_YAML); unit restarted"
        ;;
    *)
        usage
        ;;
esac
