#!/usr/bin/env bash
# Install the optional GraspNet runtime into one explicit, user-owned prefix.
# Target: JetPack 6.2.2 / L4T R36.5 / CUDA 12.6 / Python 3.10 / Jetson Orin.
#
# Safety properties:
#   * the default mode is read-only preflight + plan;
#   * installation requires --install and an absolute --prefix;
#   * no system package manager or privilege escalation is used;
#   * ~/.local is neither read as a Python package source nor modified;
#   * every pip command uses the prefix's dedicated virtual environment.

set -Eeuo pipefail
IFS=$'\n\t'
umask 022

# Do not let a broken ~/.local torch (or an inherited PYTHONPATH pointing at it)
# contaminate either preflight or compilation. This only affects this process.
export PYTHONNOUSERSITE=1
unset PYTHONPATH

readonly TARGET_ARCH="aarch64"
readonly TARGET_L4T_RELEASE="36"
readonly TARGET_L4T_REVISION_MAJOR="5"
readonly TARGET_JETPACK="6.2.2"
readonly TARGET_CUDA="12.6"
readonly TARGET_PYTHON="3.10"
readonly SYSTEM_PYTHON="/usr/bin/python3.10"
readonly ROS_DISTRO_TARGET="humble"
readonly CUDA_ROOT="/usr/local/cuda"
readonly MIN_FREE_KIB=$((12 * 1024 * 1024))

readonly BASELINE_URL="https://github.com/graspnet/graspnet-baseline.git"
readonly BASELINE_REF="main"
readonly API_URL="https://github.com/graspnet/graspnetAPI.git"
readonly API_REF="master"

# Posted in the NVIDIA Developer Forum JetPack 6.2 PyTorch 2.8 thread:
# https://forums.developer.nvidia.com/t/pytorch-2-8-wheel-for-jetpack-6-2/341339
readonly TORCH_WHEEL_URL="https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl"
readonly TORCH_WHEEL_SHA256="62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc"
readonly TORCH_WHEEL_NAME="torch-2.8.0-cp310-cp310-linux_aarch64.whl"

# Official RealSense checkpoint linked by graspnet-baseline's README.
readonly CHECKPOINT_ID="1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk"
readonly CHECKPOINT_SHARE_URL="https://drive.google.com/file/d/${CHECKPOINT_ID}/view?usp=sharing"
readonly CHECKPOINT_CURL_URL="https://drive.usercontent.google.com/download?id=${CHECKPOINT_ID}&export=download&confirm=t"
readonly CHECKPOINT_SHA256="60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868"

MODE="preflight"
PREFIX=""
JOBS=""
ACTIVE_DOWNLOAD=""

log() {
    printf '[graspnet-setup] %s\n' "$*"
}

warn() {
    printf '[graspnet-setup] WARNING: %s\n' "$*" >&2
}

die() {
    printf '[graspnet-setup] ERROR: %s\n' "$*" >&2
    exit 1
}

usage() {
    printf '%s\n' \
        'Usage:' \
        '  setup_graspnet_jetson.sh --prefix /absolute/path [--preflight-only]' \
        '  setup_graspnet_jetson.sh --prefix /absolute/path --install [--jobs N]' \
        '  setup_graspnet_jetson.sh --prefix /absolute/path --verify-only' \
        '' \
        'Modes:' \
        '  --preflight-only  Read-only host checks and exact installation plan (default).' \
        '  --install         Create the isolated runtime, build CUDA ops, and verify it.' \
        '  --verify-only     Re-run verification against an existing prefix.' \
        '' \
        'The prefix must be explicit, absolute, user-writable, and outside system/ROS paths.'
}

cleanup_download() {
    if [[ -n "$ACTIVE_DOWNLOAD" && -f "$ACTIVE_DOWNLOAD" ]]; then
        case "$ACTIVE_DOWNLOAD" in
            "$PREFIX"/downloads/.download.*)
                rm -f -- "$ACTIVE_DOWNLOAD"
                ;;
            *)
                warn "refusing to clean unexpected temporary path: $ACTIVE_DOWNLOAD"
                ;;
        esac
    fi
}
trap cleanup_download EXIT

parse_args() {
    while (($#)); do
        case "$1" in
            --prefix)
                (($# >= 2)) || die "--prefix needs a path"
                PREFIX=$2
                shift 2
                ;;
            --jobs)
                (($# >= 2)) || die "--jobs needs a positive integer"
                JOBS=$2
                shift 2
                ;;
            --preflight-only)
                MODE="preflight"
                shift
                ;;
            --install)
                MODE="install"
                shift
                ;;
            --verify-only)
                MODE="verify"
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                die "unknown argument: $1"
                ;;
        esac
    done
}

validate_prefix() {
    [[ -n "$PREFIX" ]] || die "--prefix is required (there is deliberately no implicit target)"
    [[ "$PREFIX" == /* ]] || die "--prefix must be absolute: $PREFIX"
    [[ "$PREFIX" =~ ^/[[:alnum:]_.@/+~-]+$ ]] || \
        die "--prefix may only contain letters, digits, /, ., _, @, +, ~, and -"

    PREFIX=$(readlink -m -- "$PREFIX")
    case "$PREFIX" in
        /|/bin|/bin/*|/boot|/boot/*|/dev|/dev/*|/etc|/etc/*|/lib|/lib/*|\
        /lib64|/lib64/*|/proc|/proc/*|/root|/root/*|/run|/run/*|/sbin|/sbin/*|\
        /sys|/sys/*|/usr|/usr/*|/var|/var/*|/opt/ros|/opt/ros/*)
            die "refusing system or ROS prefix: $PREFIX"
            ;;
    esac
}

set_paths() {
    VENV_DIR="$PREFIX/venv"
    VENV_PYTHON="$VENV_DIR/bin/python"
    DOWNLOAD_DIR="$PREFIX/downloads"
    SOURCE_DIR="$PREFIX/src"
    BASELINE_DIR="$SOURCE_DIR/graspnet-baseline"
    API_DIR="$SOURCE_DIR/graspnetAPI"
    CHECKPOINT_PATH="$PREFIX/checkpoint-rs.tar"
    TORCH_WHEEL_PATH="$DOWNLOAD_DIR/$TORCH_WHEEL_NAME"
    CONSTRAINTS_PATH="$PREFIX/constraints-jp622.txt"
    ENV_FILE="$PREFIX/activate_om6dof_graspnet.sh"
    MANIFEST_PATH="$PREFIX/versions.txt"
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "missing required command: $1"
}

nearest_existing_parent() {
    local probe=$1
    local parent
    while [[ ! -e "$probe" ]]; do
        parent=$(dirname -- "$probe")
        [[ "$parent" != "$probe" ]] || break
        probe=$parent
    done
    [[ -d "$probe" ]] || die "cannot find an existing parent for $1"
    printf '%s\n' "$probe"
}

preflight_host() {
    local command_name
    for command_name in awk chmod cmake curl date df dirname dpkg-query env file \
        flock g++ gcc git grep head make mkdir mktemp mv nproc readlink rm sed \
        sha256sum sleep stat tr; do
        require_command "$command_name"
    done

    local arch
    arch=$(uname -m)
    [[ "$arch" == "$TARGET_ARCH" ]] || \
        die "architecture is $arch; this recipe is only for $TARGET_ARCH Jetson Orin"

    [[ -r /etc/os-release ]] || die "/etc/os-release is unavailable"
    local os_id os_version
    os_id=$(sed -n 's/^ID=//p' /etc/os-release | tr -d '"')
    os_version=$(sed -n 's/^VERSION_ID=//p' /etc/os-release | tr -d '"')
    [[ "$os_id" == "ubuntu" && "$os_version" == "22.04" ]] || \
        die "expected Ubuntu 22.04, found ${os_id:-unknown} ${os_version:-unknown}"

    [[ -r /etc/nv_tegra_release ]] || \
        die "/etc/nv_tegra_release is missing; this does not look like Jetson Linux"
    local l4t_line l4t_release l4t_revision
    l4t_line=$(head -n 1 /etc/nv_tegra_release)
    if [[ "$l4t_line" =~ ^#[[:space:]]R([0-9]+).*REVISION:[[:space:]]([0-9.]+) ]]; then
        l4t_release=${BASH_REMATCH[1]}
        l4t_revision=${BASH_REMATCH[2]}
    else
        die "cannot parse L4T release: $l4t_line"
    fi
    [[ "$l4t_release" == "$TARGET_L4T_RELEASE" && \
       "$l4t_revision" =~ ^${TARGET_L4T_REVISION_MAJOR}([.]|$) ]] || \
        die "expected L4T R36.5.x, found R${l4t_release}.${l4t_revision}"
    log "L4T R${l4t_release}.${l4t_revision}"

    local jetpack_version
    if jetpack_version=$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null); then
        [[ "$jetpack_version" == "$TARGET_JETPACK"* ]] || \
            die "nvidia-jetpack is $jetpack_version; expected $TARGET_JETPACK"
        log "JetPack metapackage $jetpack_version"
    else
        warn "nvidia-jetpack metapackage is absent; accepting the exact L4T R36.5 gate"
    fi

    [[ -x "$SYSTEM_PYTHON" ]] || die "$SYSTEM_PYTHON is missing"
    local python_version
    python_version=$("$SYSTEM_PYTHON" -s -c \
        'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    [[ "$python_version" == "$TARGET_PYTHON" ]] || \
        die "expected Python $TARGET_PYTHON, found $python_version"
    "$SYSTEM_PYTHON" -s -m venv --help >/dev/null 2>&1 || \
        die "Python venv support is missing; ask the system administrator for python3.10-venv"
    "$SYSTEM_PYTHON" -s -m ensurepip --version >/dev/null 2>&1 || \
        die "Python ensurepip is missing; ask the system administrator for python3.10-venv"
    [[ -r /usr/include/python3.10/Python.h ]] || \
        die "Python headers are missing; ask the system administrator for python3.10-dev"
    "$SYSTEM_PYTHON" -s -c 'import cv2' >/dev/null 2>&1 || \
        die "system cv2 is missing; install the Ubuntu Python OpenCV package before continuing"

    [[ -x "$CUDA_ROOT/bin/nvcc" ]] || die "$CUDA_ROOT/bin/nvcc is missing"
    [[ -r "$CUDA_ROOT/include/cuda_runtime.h" ]] || \
        die "CUDA development headers are missing under $CUDA_ROOT/include"
    local nvcc_version
    nvcc_version=$("$CUDA_ROOT/bin/nvcc" --version)
    [[ "$nvcc_version" == *"release $TARGET_CUDA"* ]] || \
        die "expected CUDA $TARGET_CUDA from $CUDA_ROOT, got: ${nvcc_version//$'\n'/ }"

    [[ -r "/opt/ros/$ROS_DISTRO_TARGET/setup.bash" ]] || \
        die "ROS 2 $ROS_DISTRO_TARGET is not installed under /opt/ros"

    local storage_path free_kib free_gib
    storage_path=$(nearest_existing_parent "$PREFIX")
    free_kib=$(df -Pk -- "$storage_path" | awk 'NR == 2 {print $4}')
    [[ "$free_kib" =~ ^[0-9]+$ ]] || die "cannot determine free space at $storage_path"
    free_gib=$(awk -v kib="$free_kib" 'BEGIN {printf "%.1f", kib/1024/1024}')
    ((free_kib >= MIN_FREE_KIB)) || \
        die "only ${free_gib} GiB free at $storage_path; at least 12 GiB is required"
    log "free space ${free_gib} GiB at $storage_path"

    local user_site
    user_site=$("$SYSTEM_PYTHON" -s -c 'import site; print(site.getusersitepackages())')
    if [[ -e "$user_site/torch" || -e "$user_site/torch.py" ]]; then
        warn "found user-site torch at $user_site; it will be ignored and left untouched"
    else
        log "user site is disabled for this setup: $user_site"
    fi

    if [[ -z "$JOBS" ]]; then
        JOBS=$(nproc)
        ((JOBS > 4)) && JOBS=4
    fi
    [[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer"
    ((JOBS <= 16)) || die "--jobs above 16 is refused to avoid Jetson memory pressure"
}

print_plan() {
    printf '%s\n' \
        '' \
        'Read-only preflight passed. Planned writes (only with --install):' \
        "  prefix:              $PREFIX" \
        "  venv:                $VENV_DIR (--system-site-packages)" \
        "  graspnet-baseline:   $BASELINE_DIR ($BASELINE_REF)" \
        "  graspnetAPI:         $API_DIR ($API_REF)" \
        "  CUDA build jobs:     $JOBS (Orin arch 8.7)" \
        "  PyTorch wheel:       $TORCH_WHEEL_URL" \
        "  PyTorch SHA-256:      $TORCH_WHEEL_SHA256" \
        "  RealSense checkpoint:$CHECKPOINT_SHARE_URL" \
        "  activation helper:   $ENV_FILE"
}

pip_install() {
    local attempt=1
    while true; do
        if "$VENV_PYTHON" -m pip --isolated install \
            --disable-pip-version-check \
            --cache-dir "$PREFIX/pip-cache" \
            --retries 5 \
            --timeout 60 \
            --progress-bar off \
            --index-url https://pypi.org/simple \
            --constraint "$CONSTRAINTS_PATH" \
            "$@"; then
            return 0
        fi
        ((attempt >= 5)) && return 1
        warn "pip transaction failed (attempt $attempt/5); retrying in 2 s"
        ((attempt += 1))
        sleep 2
    done
}

prepare_prefix_and_venv() {
    mkdir -p -- "$PREFIX" "$DOWNLOAD_DIR" "$SOURCE_DIR"

    # A concurrent setup in the same prefix could corrupt an extension build.
    exec {LOCK_FD}>"$PREFIX/.setup.lock"
    flock -n "$LOCK_FD" || die "another setup process holds $PREFIX/.setup.lock"

    if [[ ! -x "$VENV_PYTHON" ]]; then
        log "creating Python $TARGET_PYTHON venv at $VENV_DIR"
        "$SYSTEM_PYTHON" -s -m venv --system-site-packages "$VENV_DIR"
    else
        grep -Eq '^include-system-site-packages = true$' "$VENV_DIR/pyvenv.cfg" || \
            die "existing venv was not created with --system-site-packages: $VENV_DIR"
        local existing_version
        existing_version=$("$VENV_PYTHON" -s -c \
            'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
        [[ "$existing_version" == "$TARGET_PYTHON" ]] || \
            die "existing venv uses Python $existing_version, not $TARGET_PYTHON"
        log "reusing venv $VENV_DIR"
    fi

    local constraints_tmp="$CONSTRAINTS_PATH.tmp"
    printf '%s\n' \
        '# GraspNet/Open3D compatibility and ROS cv_bridge ABI guard.' \
        'numpy==1.26.4' > "$constraints_tmp"
    mv -- "$constraints_tmp" "$CONSTRAINTS_PATH"

    pip_install --upgrade 'pip<25' 'setuptools<75' wheel
}

download_torch_wheel() {
    if [[ -f "$TORCH_WHEEL_PATH" ]]; then
        printf '%s  %s\n' "$TORCH_WHEEL_SHA256" "$TORCH_WHEEL_PATH" | \
            sha256sum --check --status || \
            die "existing PyTorch wheel has the wrong SHA-256: $TORCH_WHEEL_PATH"
        log "reusing verified PyTorch wheel"
        return
    fi

    ACTIVE_DOWNLOAD=$(mktemp "$DOWNLOAD_DIR/.download.torch.XXXXXX")
    log "downloading NVIDIA Forum PyTorch 2.8 wheel"
    curl --proto '=https' --tlsv1.2 --fail --location \
        --retry 3 --retry-all-errors \
        --output "$ACTIVE_DOWNLOAD" "$TORCH_WHEEL_URL"
    printf '%s  %s\n' "$TORCH_WHEEL_SHA256" "$ACTIVE_DOWNLOAD" | \
        sha256sum --check --status || die "PyTorch wheel SHA-256 mismatch"
    mv -- "$ACTIVE_DOWNLOAD" "$TORCH_WHEEL_PATH"
    ACTIVE_DOWNLOAD=""
}

install_torch_and_dependencies() {
    pip_install 'numpy==1.26.4' 'Cython<3' packaging typing-extensions \
        filelock fsspec jinja2 networkx 'sympy>=1.13.3' gdown
    pip_install --no-deps "$TORCH_WHEEL_PATH"

    log "checking PyTorch CUDA before compiling extensions"
    "$VENV_PYTHON" -s - <<'PY'
import torch

assert torch.__version__.split("+")[0] == "2.8.0", torch.__version__
assert torch.version.cuda == "12.6", torch.version.cuda
assert torch.cuda.is_available(), "torch.cuda.is_available() is false"
capability = torch.cuda.get_device_capability(0)
assert capability == (8, 7), f"expected Orin sm_87, got {capability}"
x = torch.arange(1024, dtype=torch.float32, device="cuda")
assert float((x * x).sum().cpu()) > 0.0
torch.cuda.synchronize()
print(f"torch={torch.__version__} cuda={torch.version.cuda} "
      f"gpu={torch.cuda.get_device_name(0)} capability={capability}")
PY

    # Baseline requirements, excluding torch (the CUDA wheel above is the only
    # allowed torch source). Open3D's official ARM64 wheel is CPU-only, which is
    # sufficient for the baseline's point-cloud downsampling/collision helper.
    pip_install tensorboard scipy 'open3d>=0.18,<0.20' Pillow tqdm \
        'pyrealsense2==2.58.2.10647'

    # Runtime imports reached eagerly by graspnetAPI.__init__. Keep OpenCV from
    # Ubuntu/system-site so the ROS image stack is not replaced by a pip wheel.
    pip_install transforms3d trimesh matplotlib pywavefront scikit-image \
        cvxopt dill h5py scikit-learn grasp_nms IPython ruamel.yaml \
        multiprocess setproctitle colorlog
    # Install the package code without pulling pip OpenCV over Ubuntu cv2.
    pip_install --no-deps autolab-core
    pip_install --no-deps autolab-perception
}

clone_or_reuse() {
    local url=$1
    local ref=$2
    local destination=$3
    local label=$4

    if [[ ! -e "$destination" ]]; then
        log "cloning $label ($ref)"
        git clone --depth 1 --branch "$ref" --single-branch \
            "$url" "$destination"
        return
    fi

    [[ -d "$destination/.git" ]] || \
        die "$destination exists but is not a git checkout"
    local origin
    origin=$(git -C "$destination" remote get-url origin)
    [[ "$origin" == "$url" ]] || \
        die "$label checkout has unexpected origin $origin (expected $url)"
    git -C "$destination" diff --quiet --ignore-submodules -- || \
        die "$label checkout has local changes; refusing to overwrite them"
    git -C "$destination" diff --cached --quiet --ignore-submodules -- || \
        die "$label checkout has staged changes; refusing to overwrite them"
    log "reusing $label at commit $(git -C "$destination" rev-parse --short HEAD)"
}

install_sources_and_extensions() {
    clone_or_reuse "$BASELINE_URL" "$BASELINE_REF" "$BASELINE_DIR" \
        "graspnet-baseline"
    clone_or_reuse "$API_URL" "$API_REF" "$API_DIR" "graspnetAPI"

    [[ -f "$BASELINE_DIR/pointnet2/setup.py" ]] || \
        die "pointnet2/setup.py is missing from graspnet-baseline"
    [[ -f "$BASELINE_DIR/knn/setup.py" ]] || \
        die "knn/setup.py is missing from graspnet-baseline"

    # Install API code without its broad evaluation dependency set: the eager
    # import/runtime subset needed by this adapter was installed explicitly.
    pip_install --no-deps --no-build-isolation -e "$API_DIR"

    export CUDA_HOME="$CUDA_ROOT"
    export PATH="$CUDA_ROOT/bin:$PATH"
    export LD_LIBRARY_PATH="$CUDA_ROOT/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export TORCH_CUDA_ARCH_LIST="8.7"
    export FORCE_CUDA="1"
    export MAX_JOBS="$JOBS"
    export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"

    if PYTHONPATH="$BASELINE_DIR" "$VENV_PYTHON" -s -c \
            'import torch; import pointnet2._ext, knn_pytorch.knn_pytorch' \
            >/dev/null 2>&1; then
        log "reusing importable pointnet2 and knn CUDA extensions"
    else
        log "building pointnet2 CUDA extension (sm_87, jobs=$JOBS)"
        pip_install --force-reinstall --no-deps --no-build-isolation \
            "$BASELINE_DIR/pointnet2"
        log "building knn CUDA extension (sm_87, jobs=$JOBS)"
        pip_install --force-reinstall --no-deps --no-build-isolation \
            "$BASELINE_DIR/knn"
    fi
}

checkpoint_is_plausible() {
    local path=$1
    [[ -f "$path" ]] || return 1
    local bytes
    bytes=$(stat -c '%s' "$path")
    # checkpoint-rs.tar is a ~12.5 MB PyTorch zip archive. Check both its
    # pinned content hash and MIME type so a Drive error page cannot pass.
    ((bytes >= 10 * 1024 * 1024)) || return 1
    [[ $(file --brief --mime-type "$path") != "text/html" ]] || return 1
    printf '%s  %s\n' "$CHECKPOINT_SHA256" "$path" | \
        sha256sum --check --status
}

download_checkpoint() {
    if [[ -f "$CHECKPOINT_PATH" ]]; then
        checkpoint_is_plausible "$CHECKPOINT_PATH" || \
            die "existing checkpoint is too small or HTML: $CHECKPOINT_PATH"
        log "reusing existing checkpoint $(sha256sum "$CHECKPOINT_PATH" | awk '{print $1}')"
        return
    fi

    ACTIVE_DOWNLOAD=$(mktemp "$DOWNLOAD_DIR/.download.checkpoint.XXXXXX")
    log "downloading official checkpoint-rs.tar with gdown"
    # gdown 6 accepts a Drive ID directly and removed the older --fuzzy flag.
    # Passing the ID works on both old and new releases and avoids scraping the
    # share page before following Google's large-file confirmation flow.
    if ! "$VENV_PYTHON" -s -m gdown "$CHECKPOINT_ID" \
        --output "$ACTIVE_DOWNLOAD"; then
        warn "gdown failed; trying the official Google Drive file ID with curl"
        curl --proto '=https' --tlsv1.2 --fail --location \
            --retry 3 --retry-all-errors \
            --output "$ACTIVE_DOWNLOAD" "$CHECKPOINT_CURL_URL"
    fi
    checkpoint_is_plausible "$ACTIVE_DOWNLOAD" || \
        die "checkpoint download is too small or is an HTML response"
    mv -- "$ACTIVE_DOWNLOAD" "$CHECKPOINT_PATH"
    ACTIVE_DOWNLOAD=""
    log "checkpoint SHA-256 $(sha256sum "$CHECKPOINT_PATH" | awk '{print $1}')"
}

runtime_pythonpath() {
    local venv_site
    venv_site=$("$VENV_PYTHON" -s -c \
        'import sysconfig; print(sysconfig.get_paths()["purelib"])')
    printf '%s:%s:%s:%s:%s:%s\n' \
        "$venv_site" "$API_DIR" "$BASELINE_DIR" \
        "$BASELINE_DIR/models" "$BASELINE_DIR/utils" "$BASELINE_DIR/dataset"
}

verify_runtime() {
    [[ -x "$VENV_PYTHON" ]] || die "missing environment: $VENV_PYTHON"
    [[ -f "$CHECKPOINT_PATH" ]] || die "missing checkpoint: $CHECKPOINT_PATH"
    [[ -d "$BASELINE_DIR" && -d "$API_DIR" ]] || \
        die "missing GraspNet source checkout under $SOURCE_DIR"

    local model_pythonpath
    model_pythonpath=$(runtime_pythonpath)
    export CUDA_HOME="$CUDA_ROOT"
    export LD_LIBRARY_PATH="$CUDA_ROOT/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$model_pythonpath"
    export GRASPNET_REPO_PATH="$BASELINE_DIR"
    export GRASPNET_CHECKPOINT="$CHECKPOINT_PATH"

    log "verifying CUDA extensions, upstream imports, and checkpoint load"
    "$VENV_PYTHON" -s - <<'PY'
import os
import torch
import pyrealsense2  # noqa: F401

import pointnet2._ext  # noqa: F401
import knn_pytorch.knn_pytorch  # noqa: F401
from collision_detector import ModelFreeCollisionDetector  # noqa: F401
from graspnet import GraspNet, pred_decode  # noqa: F401
from graspnetAPI import GraspGroup  # noqa: F401

assert torch.__version__.split("+")[0] == "2.8.0", torch.__version__
assert torch.version.cuda == "12.6", torch.version.cuda
assert torch.cuda.is_available(), "CUDA unavailable"

checkpoint = torch.load(
    os.environ["GRASPNET_CHECKPOINT"], map_location="cpu", weights_only=True)
state = checkpoint.get("model_state_dict") if isinstance(checkpoint, dict) else None
assert isinstance(state, dict), "checkpoint has no model_state_dict"
net = GraspNet(
    input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
    cylinder_radius=0.05, hmin=-0.02,
    hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
net.load_state_dict(state, strict=True)
del checkpoint, state
net = net.eval().to("cuda")
probe = torch.linspace(0.0, 1.0, 4096, device="cuda")
value = float(torch.linalg.vector_norm(probe).cpu())
torch.cuda.synchronize()
assert value > 0.0
del net, probe
torch.cuda.empty_cache()
print("verified: torch CUDA tensor runtime, pointnet2, knn, graspnetAPI, "
      "pred_decode, collision detector, and checkpoint state")
PY

    # ROS console scripts commonly retain /usr/bin/python3 in their shebang.
    # Prove that the generated PYTHONPATH exposes this isolated ABI-compatible
    # runtime to that interpreter without enabling ~/.local.
    log "verifying the /usr/bin/python3 ROS entrypoint view"
    env PYTHONNOUSERSITE=1 PYTHONPATH="$model_pythonpath" \
        CUDA_HOME="$CUDA_ROOT" \
        LD_LIBRARY_PATH="$CUDA_ROOT/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
        /usr/bin/python3 -s -c \
        'import torch, pyrealsense2, pointnet2._ext, knn_pytorch.knn_pytorch; from graspnetAPI import GraspGroup; from graspnet import GraspNet, pred_decode; from collision_detector import ModelFreeCollisionDetector; assert torch.cuda.is_available(); print(torch.__file__)'
}

write_activation_helper() {
    local venv_site helper_tmp
    venv_site=$("$VENV_PYTHON" -s -c \
        'import sysconfig; print(sysconfig.get_paths()["purelib"])')
    helper_tmp="$ENV_FILE.tmp"
    # Dollar expressions below must remain literal for the generated helper to
    # evaluate when it is sourced, not while this setup script writes it.
    # shellcheck disable=SC2016
    {
        printf '%s\n' \
            '# Generated by setup_graspnet_jetson.sh.' \
            '# Source ROS first, then this file. It filters explicit ~/.local paths.' \
            'export PYTHONNOUSERSITE=1'
        printf 'export OM6DOF_GRASPNET_PREFIX=%q\n' "$PREFIX"
        printf 'export OM6DOF_GRASPNET_VENV=%q\n' "$VENV_DIR"
        printf 'export GRASPNET_REPO_PATH=%q\n' "$BASELINE_DIR"
        printf 'export GRASPNET_CHECKPOINT=%q\n' "$CHECKPOINT_PATH"
        printf 'export CUDA_HOME=%q\n' "$CUDA_ROOT"
        printf 'export PATH=%q:"${PATH}"\n' "$VENV_DIR/bin"
        printf 'export LD_LIBRARY_PATH=%q"${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"\n' \
            "$CUDA_ROOT/lib64"
        printf '%s\n' \
            '_om6dof_filtered_pythonpath=""' \
            'IFS=":" read -r -a _om6dof_python_entries <<< "${PYTHONPATH-}"' \
            'for _om6dof_python_entry in "${_om6dof_python_entries[@]}"; do' \
            '    [[ -n "$_om6dof_python_entry" ]] || continue' \
            '    case "$_om6dof_python_entry" in' \
            '        "${HOME:-/__no_home__}/.local/"*) continue ;;' \
            '    esac' \
            '    _om6dof_filtered_pythonpath="${_om6dof_filtered_pythonpath:+${_om6dof_filtered_pythonpath}:}${_om6dof_python_entry}"' \
            'done'
        printf 'export PYTHONPATH=%q:%q:%q:%q:%q:%q"${_om6dof_filtered_pythonpath:+:${_om6dof_filtered_pythonpath}}"\n' \
            "$venv_site" "$API_DIR" "$BASELINE_DIR" \
            "$BASELINE_DIR/models" "$BASELINE_DIR/utils" "$BASELINE_DIR/dataset"
        printf '%s\n' \
            'unset _om6dof_filtered_pythonpath _om6dof_python_entries _om6dof_python_entry'
    } > "$helper_tmp"
    mv -- "$helper_tmp" "$ENV_FILE"
    chmod 0644 "$ENV_FILE"
}

write_manifest() {
    local manifest_tmp="$MANIFEST_PATH.tmp"
    local checkpoint_sha torch_version
    checkpoint_sha=$(sha256sum "$CHECKPOINT_PATH" | awk '{print $1}')
    torch_version=$("$VENV_PYTHON" -s -c 'import torch; print(torch.__version__)')
    {
        printf 'created_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        printf 'target=JetPack-%s_L4T-R36.5_CUDA-%s_Python-%s_aarch64\n' \
            "$TARGET_JETPACK" "$TARGET_CUDA" "$TARGET_PYTHON"
        printf 'torch_version=%s\n' "$torch_version"
        printf 'torch_wheel_url=%s\n' "$TORCH_WHEEL_URL"
        printf 'torch_wheel_sha256=%s\n' "$TORCH_WHEEL_SHA256"
        printf 'graspnet_baseline_commit=%s\n' \
            "$(git -C "$BASELINE_DIR" rev-parse HEAD)"
        printf 'graspnet_api_commit=%s\n' \
            "$(git -C "$API_DIR" rev-parse HEAD)"
        printf 'checkpoint_source=%s\n' "$CHECKPOINT_SHARE_URL"
        printf 'checkpoint_expected_sha256=%s\n' "$CHECKPOINT_SHA256"
        printf 'checkpoint_sha256=%s\n' "$checkpoint_sha"
    } > "$manifest_tmp"
    mv -- "$manifest_tmp" "$MANIFEST_PATH"
}

main() {
    parse_args "$@"
    validate_prefix
    set_paths
    preflight_host

    case "$MODE" in
        preflight)
            print_plan
            printf '%s\n' \
                '' \
                'No installation has been performed. Re-run the same command' \
                'with --install only after reviewing the plan above.'
            ;;
        verify)
            verify_runtime
            log "existing runtime verified; no files changed"
            ;;
        install)
            print_plan
            log "--install explicitly authorized; writes remain confined to $PREFIX"
            prepare_prefix_and_venv
            download_torch_wheel
            install_torch_and_dependencies
            install_sources_and_extensions
            download_checkpoint
            verify_runtime
            write_activation_helper
            write_manifest
            printf '%s\n' \
                '' \
                'GraspNet runtime is ready.' \
                'For ROS: source /opt/ros/humble/setup.bash, source the workspace' \
                "overlay, then source $ENV_FILE before ros2 launch." \
                "Set graspnet_repo_path to $BASELINE_DIR" \
                "and graspnet_checkpoint to $CHECKPOINT_PATH in the selected params file."
            ;;
        *)
            die "internal mode error: $MODE"
            ;;
    esac
}

main "$@"
