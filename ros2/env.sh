#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case $(uname -m) in
  x86_64) export HOST_ARCH=amd64 ;;
  aarch64|arm64) export HOST_ARCH=arm64 ;;
  *) echo 'Only amd64 and arm64 hosts are supported.' >&2; exit 2 ;;
esac
export LOCAL_UID=$(id -u) LOCAL_GID=$(id -g)
upstream="$root/workspace/upstream"
mkdir -p "$root/workspace/_out/home" "$upstream/SchurVINS"
compose=(docker compose -f "$root/compose.yaml")
action=${1:-help}
shift || true
case "$action" in
  fetch)
    while IFS=$'\t' read -r name url revision; do
      checkout="$upstream/$name"
      if [[ ! -e "$checkout" ]]; then
        # Clone and verify in a temporary sibling so an interrupted fetch leaves the target reusable.
        staging=$(mktemp -d "$upstream/.${name}.XXXXXX")
        git clone "$url" "$staging"
        git -C "$staging" checkout --detach "$revision"
        mv -T -- "$staging" "$checkout"
      fi
      if [[ $(git -C "$checkout" rev-parse --show-toplevel) != "$checkout" ||
            $(git -C "$checkout" rev-parse HEAD) != "$revision" ||
            -n $(git -C "$checkout" status --porcelain) ]]; then
        echo "Dependency $checkout differs from the pinned clean checkout $revision" >&2
        exit 1
      fi
    done < "$root/dependencies.tsv"
    ;;
  config) exec "${compose[@]}" config "$@" ;;
  build-image) exec "${compose[@]}" build vio "$@" ;;
  build) exec "$0" run bash /workspaces/schurvins_ws/scripts/build.sh "$@" ;;
  run) exec "${compose[@]}" run --rm -T vio "$@" ;;
  *) echo 'Usage: ros2/env.sh {fetch|config|build-image|build|run COMMAND...}'; exit 2 ;;
esac
