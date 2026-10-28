#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
work_dir=$(mktemp -d /tmp/chassis-control-check.XXXXXX)
trap 'rm -rf "$work_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -I"$project_dir/application" \
  "$project_dir/tests/chassis_control/behaviour_test.c" \
  "$project_dir/application/chassis_behaviour.c" \
  "$project_dir/application/host_command.c" -lm -o "$work_dir/behaviour"
"$work_dir/behaviour"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -I"$project_dir/tests/chassis_control/stubs" \
  -I"$project_dir/application" -I"$project_dir/components/controller" \
  "$project_dir/tests/chassis_control/task_test.c" \
  "$project_dir/application/chassis_behaviour.c" \
  "$project_dir/application/chassis_task.c" \
  "$project_dir/components/controller/pid.c" -lm -o "$work_dir/task"
"$work_dir/task"
