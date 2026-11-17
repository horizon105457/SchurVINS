#!/usr/bin/env bash
set -euo pipefail
if [[ ${HEXBOT_VIO_CONTAINER:-0} != 1 ]]; then
  echo 'Run scripts/env.sh build on the host.' >&2; exit 2
fi
mkdir -p "$VIO_OUT"
export CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS:-2}
export MAKEFLAGS="-j${BUILD_JOBS:-2} -l${BUILD_JOBS:-2}"
cd /workspaces/schurvins_ws
colcon --log-base "$VIO_OUT/log" build --base-paths src \
  --build-base "$VIO_OUT/build" --install-base "$VIO_OUT/install" \
  --parallel-workers 1 --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
