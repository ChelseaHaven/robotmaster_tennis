#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
test_binary=$(mktemp /tmp/imu-telemetry-test.XXXXXX)
trap 'rm -f "$test_binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
 -ffunction-sections -fdata-sections -Wl,--gc-sections \
 -I"$project_dir/tests/imu_telemetry/stubs" -I"$project_dir/application" \
 -I"$project_dir/bsp/boards" -I"$project_dir/components/algorithm" \
 "$project_dir/tests/imu_telemetry/test.c" \
 "$project_dir/application/host_comm_task.c" \
 "$project_dir/application/host_protocol.c" \
 "$project_dir/components/algorithm/CRC8_CRC16.c" -o "$test_binary"
"$test_binary"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
 -ffunction-sections -fdata-sections -Wl,--gc-sections \
 -I"$project_dir/application" -I"$project_dir/components/devices" \
 "$project_dir/tests/imu_telemetry/magnetometer_test.c" \
 "$project_dir/components/devices/ist8310driver.c" -o "$test_binary"
"$test_binary"
