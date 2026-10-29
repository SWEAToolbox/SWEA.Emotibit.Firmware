#!/bin/bash
# Host unit test for SweaDisplayModel.h with g++ (Linux/macOS/MinGW).
set -e
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++17 -Wall -Wextra -o build/test_display_model test_display_model.cpp
./build/test_display_model
