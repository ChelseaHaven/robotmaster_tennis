#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
test_binary=$(mktemp /tmp/gyro-calibration-trigger.XXXXXX)
trap 'rm -f "$test_binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
 -I"$project_dir/tests/chassis_control/stubs" -I"$project_dir/application" -I"$project_dir/bsp/boards" \
 "$project_dir/tests/gyro_calibration_trigger/test.c" "$project_dir/application/calibrate_task.c" \
 -o "$test_binary"
"$test_binary"
