@echo off

cmake -S . -B build -A x64
cmake --build build --config Release