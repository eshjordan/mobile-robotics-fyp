#!/bin/bash

# Determine the workspace directory
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
WS_DIR="${SCRIPT_DIR}/.."

# Copy the clang-format and clang-tidy config files from the root of the workspace to all packages (unless excluded)
find ${WS_DIR}/src -mindepth 1 -maxdepth 1 -type d \
    -a ! -name "webots_ros2" \
    -exec bash -c "
    cp -f ${WS_DIR}/.clang-format {} &&
    cp -f ${WS_DIR}/.clang-tidy {} &&
    ln -sf ${WS_DIR}/build/compile_commands.json {}/compile_commands.json
    " \;

