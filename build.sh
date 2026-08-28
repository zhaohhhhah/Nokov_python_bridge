#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$SCRIPT_DIR"

g++ \
  -D_LINUX \
  -std=c++17 \
  -O2 \
  -fPIC \
  -shared \
  nokov_bridge.cpp \
  -I"$SDK_ROOT/include" \
  -L"$SDK_ROOT/lib" \
  -lnokov_sdk \
  -pthread \
  -Wl,-rpath,'$ORIGIN/../lib' \
  -o libnokov_pybridge.so

echo "Built: $SCRIPT_DIR/libnokov_pybridge.so"
ldd "$SCRIPT_DIR/libnokov_pybridge.so"
