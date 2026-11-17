#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -n ${XDG_RUNTIME_DIR:-} ]]; then
  mkdir -p "$XDG_RUNTIME_DIR"
  chmod 700 "$XDG_RUNTIME_DIR"
fi
case $(uname -m) in
  x86_64) arch=amd64 ;;
  aarch64|arm64) arch=arm64 ;;
  *) exit 2 ;;
esac
export VIO_OUT=/workspaces/schurvins_ws/_out/$arch/jazzy-v1
if [[ -f "$VIO_OUT/install/schurvins_jazzy/share/schurvins_jazzy/local_setup.bash" ]]; then
  source "$VIO_OUT/install/setup.bash"
fi
SIM_OUT=/workspaces/simulation_ws/_out/$arch/jazzy-v1
if [[ -f "$SIM_OUT/install/holoocean_main/share/holoocean_main/package.bash" ]]; then
  source "$SIM_OUT/install/setup.bash"
fi
exec "$@"
