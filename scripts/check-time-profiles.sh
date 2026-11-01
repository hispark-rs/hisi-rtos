#!/usr/bin/env bash
# Run in an independent checkout; optional Cargo global arguments select an
# explicit remote development overlay. No parent patch or hardware access.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "$(cargo locate-project --workspace --message-format plain)" != "$PWD/Cargo.toml" ]]; then
    echo 'Run this check from an independent hisi-rtos checkout' >&2
    exit 1
fi
mkdir -p target/time-profile-check
cargo "$@" test --locked --target x86_64-unknown-linux-gnu
cargo "$@" test --locked --target x86_64-unknown-linux-gnu --features embassy
for chip in chip-ws53 chip-ws63 chip-bs21; do
    features="embassy-only,unstable,$chip"
    cargo "$@" test --locked --target x86_64-unknown-linux-gnu --features "$features"
    cargo "$@" clippy --locked -Zbuild-std=core,alloc --target riscv32imfc-unknown-none-elf --features "$features,defmt" -- -D warnings
    cargo "$@" tree --locked --target riscv32imfc-unknown-none-elf --features "$features" --edges normal --prefix none > target/time-profile-check/dependencies.txt
    if grep -Eq '^(embassy-executor|embassy-time-queue-utils|hisi-embassy) ' target/time-profile-check/dependencies.txt; then
        echo 'Unexpected executor or duplicate time component dependency' >&2
        exit 1
    fi
done
cargo "$@" clippy --locked -Zbuild-std=core,alloc --target riscv32imfc-unknown-none-elf --features chip-ws63,embassy -- -D warnings
for features in embassy-only,chip-ws63 chip-ws53,unstable embassy-only,unstable,chip-ws53,chip-ws63; do
    if cargo "$@" check --locked --target x86_64-unknown-linux-gnu --features "$features" > target/time-profile-check/negative.log 2>&1; then
        echo "Invalid time profile succeeded: $features" >&2
        exit 1
    fi
    case "$features" in
        embassy-only,chip-ws63) grep -q 'embassy-only requires unstable' target/time-profile-check/negative.log ;;
        chip-ws53,unstable) grep -q 'no scheduler port is implemented' target/time-profile-check/negative.log ;;
        *) grep -q 'select exactly ONE chip feature' target/time-profile-check/negative.log ;;
    esac
done
