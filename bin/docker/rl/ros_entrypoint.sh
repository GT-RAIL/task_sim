#!/bin/bash
set -e

# setup ros environment
source "$TASKSIM_FILES_ROOT/workspace/devel/setup.bash"
exec "$@"
