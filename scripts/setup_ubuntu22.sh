#!/usr/bin/env bash

set -Eeuo pipefail

readonly OPENVINO_VERSION="${OPENVINO_VERSION:-2024.4.0}"
readonly SOPHUS_VERSION="${SOPHUS_VERSION:-1.22.10}"
readonly BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
mode="install"

log()
{
    printf '\n[RP-26Rune] %s\n' "$*"
}

die()
{
    printf '[RP-26Rune] ERROR: %s\n' "$*" >&2
    exit 1
}

usage()
{
    cat <<'EOF'
Usage:
  ./scripts/setup_ubuntu22.sh           Install dependencies and verify CMake
  ./scripts/setup_ubuntu22.sh --check   Only verify the existing environment
  ./scripts/setup_ubuntu22.sh --help    Show this help

Optional environment variables:
  OPENVINO_VERSION  OpenVINO APT package version (default: 2024.4.0)
  SOPHUS_VERSION    Sophus source tag (default: 1.22.10)
  BUILD_JOBS        Parallel jobs used while building Sophus (default: nproc)
EOF
}

for argument in "$@"; do
    case "${argument}" in
        --check)
            mode="check"
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            die "unknown argument: ${argument}"
            ;;
    esac
done

require_supported_host()
{
    [[ -r /etc/os-release ]] || die "cannot read /etc/os-release"

    # shellcheck disable=SC1091
    source /etc/os-release
    [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] ||
        die "this script supports Ubuntu 22.04 only (detected: ${PRETTY_NAME:-unknown})"
    [[ "$(dpkg --print-architecture)" == "amd64" ]] ||
        die "this script currently supports x86-64/amd64 only"
}

prepare_privilege_command()
{
    if (( EUID == 0 )); then
        root_command=()
    else
        command -v sudo >/dev/null 2>&1 || die "sudo is required for dependency installation"
        root_command=(sudo)
    fi
}

run_as_root()
{
    "${root_command[@]}" "$@"
}

install_apt_dependencies()
{
    log "Installing the C++ toolchain, OpenCV, optimization libraries, and video runtime"
    run_as_root apt-get update
    run_as_root apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        ffmpeg \
        git \
        gnupg \
        libboost-all-dev \
        libceres-dev \
        libeigen3-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        libopencv-dev \
        pkg-config
}

install_openvino()
{
    local work_dir key_file keyring_file repository_file
    work_dir="$(mktemp -d)"
    key_file="${work_dir}/openvino-signing-key.pub"
    keyring_file="${work_dir}/openvino-archive-keyring.gpg"
    repository_file="${work_dir}/intel-openvino-2024.list"

    trap 'rm -rf -- "${work_dir}"' RETURN

    log "Configuring the official OpenVINO 2024 APT repository"
    curl --fail --silent --show-error --location \
        --output "${key_file}" \
        https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    gpg --batch --yes --dearmor \
        --output "${keyring_file}" \
        "${key_file}"
    printf '%s\n' \
        'deb [signed-by=/usr/share/keyrings/intel-openvino-2024.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main' \
        > "${repository_file}"

    run_as_root install -m 0644 \
        "${keyring_file}" \
        /usr/share/keyrings/intel-openvino-2024.gpg
    run_as_root install -m 0644 \
        "${repository_file}" \
        /etc/apt/sources.list.d/intel-openvino-2024.list
    run_as_root apt-get update

    log "Installing the OpenVINO ${OPENVINO_VERSION} C++ runtime and CPU/AUTO plugins"
    run_as_root apt-get install -y --no-install-recommends \
        "libopenvino-dev-${OPENVINO_VERSION}" \
        "libopenvino-intel-cpu-plugin-${OPENVINO_VERSION}" \
        "libopenvino-auto-plugin-${OPENVINO_VERSION}"
}

installed_sophus_version_file()
{
    find \
        /usr/local/share /usr/local/lib /usr/share /usr/lib \
        -type f -name SophusConfigVersion.cmake \
        -print -quit 2>/dev/null || true
}

install_sophus()
{
    local version_file work_dir archive_file source_dir build_dir
    version_file="$(installed_sophus_version_file)"

    if [[ -n "${version_file}" ]] && grep -Eq \
        "PACKAGE_VERSION[[:space:]]+\"?${SOPHUS_VERSION//./\\.}" \
        "${version_file}"; then
        log "Sophus ${SOPHUS_VERSION} is already installed"
        return
    fi

    work_dir="$(mktemp -d)"
    archive_file="${work_dir}/sophus.tar.gz"
    source_dir="${work_dir}/Sophus-${SOPHUS_VERSION}"
    build_dir="${work_dir}/build"
    trap 'rm -rf -- "${work_dir}"' RETURN

    log "Building Sophus ${SOPHUS_VERSION} from its upstream release tag"
    curl --fail --silent --show-error --location \
        --output "${archive_file}" \
        "https://github.com/strasdat/Sophus/archive/refs/tags/${SOPHUS_VERSION}.tar.gz"
    tar -xzf "${archive_file}" -C "${work_dir}"
    cmake \
        -S "${source_dir}" \
        -B "${build_dir}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SOPHUS_EXAMPLES=OFF \
        -DBUILD_SOPHUS_TESTS=OFF \
        -DSOPHUS_INSTALL=ON \
        -DSOPHUS_USE_BASIC_LOGGING=ON
    cmake --build "${build_dir}" --parallel "${BUILD_JOBS}"
    run_as_root cmake --install "${build_dir}"
    run_as_root ldconfig
}

check_runtime_assets()
{
    local required_assets=(
        "src/app_plugin/detector/config/openvino/model-0624.onnx"
        "src/app_plugin/single_camera_manager/config/video/big_rune_demo.mp4"
        "src/app_plugin/detector/config/detect.json"
        "src/app_plugin/single_camera_manager/config/camera.json"
        "config/power_rune.json"
    )
    local relative_path

    for relative_path in "${required_assets[@]}"; do
        [[ -r "${project_root}/${relative_path}" ]] ||
            die "required runtime asset is missing: ${relative_path}"
    done
}

check_environment()
{
    local work_dir
    local required_commands=(cmake c++ ffmpeg)
    local executable

    log "Checking build tools and runtime assets"
    for executable in "${required_commands[@]}"; do
        command -v "${executable}" >/dev/null 2>&1 || die "missing command: ${executable}"
    done
    check_runtime_assets

    work_dir="$(mktemp -d)"
    trap 'rm -rf -- "${work_dir}"' RETURN

    log "Running a clean CMake configure check for all declared dependencies"
    cmake \
        -S "${project_root}" \
        -B "${work_dir}/build" \
        -DCMAKE_BUILD_TYPE=Release

    log "Checking that the bundled H.264 demo can be decoded"
    ffmpeg \
        -hide_banner \
        -loglevel error \
        -i "${project_root}/src/app_plugin/single_camera_manager/config/video/big_rune_demo.mp4" \
        -frames:v 1 \
        -f null -

    log "Environment check passed"
    printf '  CMake:    %s\n' "$(cmake --version | head -n 1)"
    printf '  Compiler: %s\n' "$(c++ --version | head -n 1)"
    printf '\nNext step:\n  cd %q\n  ./run_demo.sh\n' "${project_root}"
}

require_supported_host

if [[ "${mode}" == "install" ]]; then
    prepare_privilege_command
    install_apt_dependencies
    install_openvino
    install_sophus
fi

check_environment
