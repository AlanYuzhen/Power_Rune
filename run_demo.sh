#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s [--headless]\n' "$(basename -- "$0")"
}

case "$#" in
    0)
        ;;
    1)
        if [[ "$1" != "--headless" ]]; then
            usage >&2
            exit 2
        fi
        export QT_QPA_PLATFORM=offscreen
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_jobs="${BUILD_JOBS:-2}"

cmake -S "${project_root}" -B "${project_root}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${project_root}/build" --parallel "${build_jobs}"

cd "${project_root}"
exec "${project_root}/output/app"
