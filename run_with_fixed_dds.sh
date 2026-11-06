#!/usr/bin/env bash
set -euo pipefail

cd /home/unitree/ARMS
source /home/unitree/cyclonedds_py_0_10_fix/env.sh

exec "$@"
