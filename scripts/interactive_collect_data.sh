#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
COLLECT_SCRIPT="${SCRIPT_DIR}/collect_data_until_key.py"
COLLECT_PYTHON="/usr/bin/python3"
EXPORT_SCRIPT="${EXPORT_SCRIPT:-${WORKSPACE_ROOT}/tools/export_hdf5_episode.py}"
EXPORT_PYTHON="${EXPORT_PYTHON:-python3}"
MPL_CONFIG_DIR="${TMPDIR:-/tmp}/cobot_magic_mplconfig"

DATASET_DIR="${HOME}/data"
EXPORT_DIR="${HOME}/data_mp4"
TASK_NAME="DLO_Stretch"
FRAME_RATE=30
MAX_TIMESTEPS=600
SYNC_TIMEOUT=30
START_EPISODE=""

usage() {
    cat <<'EOF'
Usage:
  interactive_collect_data.sh [options]

Options:
  --start-episode INDEX  Suggested first episode ID (default: auto-detect).
  --dataset-dir PATH     HDF5 root directory (default: ~/data).
  --export-dir PATH      Video/diagnostic root directory (default: ~/data_mp4).
  --task-name NAME       Task subdirectory name (default: DLO_Stretch).
  --max-timesteps COUNT  Maximum timesteps per episode (default: 600).
  --frame-rate HZ        Collection and export frame rate (default: 30).
  --sync-timeout SEC     Abort if required topics stay unavailable (default: 30).
  -h, --help             Show this help message.

Interaction:
  During collection: press S to stop early and save.
  After export:       C = continue, R = recollect/replace current, Q = quit.
EOF
}

die() {
    printf 'Error: %s\n' "$*" >&2
    exit 1
}

require_option_value() {
    local option="$1"
    local remaining="$2"
    (( remaining >= 2 )) || die "${option} requires a value"
}

speak() {
    local message="$1"
    if command -v spd-say >/dev/null 2>&1; then
        spd-say --wait --language en "${message}" || true
    fi
}

check_required_topics() {
    local topic_list
    local topic
    local missing=()
    local required_topics=(
        "/camera_f/color/image_raw"
        "/camera_l/color/image_raw"
        "/camera_r/color/image_raw"
        "/master/joint_left"
        "/master/joint_right"
        "/puppet/joint_left"
        "/puppet/joint_right"
    )

    topic_list="$(ros2 topic list)"
    for topic in "${required_topics[@]}"; do
        if [[ $'\n'"${topic_list}"$'\n' != *$'\n'"${topic}"$'\n'* ]]; then
            missing+=("${topic}")
        fi
    done

    if (( ${#missing[@]} > 0 )); then
        printf 'Missing required ROS 2 topics:\n' >&2
        printf '  %s\n' "${missing[@]}" >&2
        die "start collect_stack_split_cameras.launch.py before running this script"
    fi
}

check_export_outputs() {
    local episode_dir="$1"
    local episode_stem="$2"
    local output
    local expected_outputs=(
        "${episode_stem}_cam_high.mp4"
        "${episode_stem}_cam_left_wrist.mp4"
        "${episode_stem}_cam_right_wrist.mp4"
        "${episode_stem}_combined.mp4"
        "${episode_stem}_contact_sheet.jpg"
        "${episode_stem}_qpos_action.png"
        "${episode_stem}_effort.png"
        "${episode_stem}_summary.json"
        "${episode_stem}_summary.txt"
    )

    for output in "${expected_outputs[@]}"; do
        [[ -s "${episode_dir}/${output}" ]] || return 1
    done
}

episode_is_free() {
    local episode_index="$1"
    local episode_stem="episode_${episode_index}"
    [[ ! -e "${DATASET_DIR}/${TASK_NAME}/${episode_stem}.hdf5" ]] &&
        [[ ! -e "${EXPORT_DIR}/${TASK_NAME}/${episode_stem}" ]]
}

detect_start_episode() {
    local maximum_index=-1
    local candidate
    local name
    local index

    for candidate in "${DATASET_DIR}/${TASK_NAME}"/episode_*.hdf5; do
        [[ -f "${candidate}" ]] || continue
        name="${candidate##*/}"
        if [[ "${name}" =~ ^episode_([0-9]+)\.hdf5$ ]]; then
            index=$((10#${BASH_REMATCH[1]}))
            (( index > maximum_index )) && maximum_index=${index}
        fi
    done

    for candidate in "${EXPORT_DIR}/${TASK_NAME}"/episode_*; do
        [[ -d "${candidate}" ]] || continue
        name="${candidate##*/}"
        if [[ "${name}" =~ ^episode_([0-9]+)$ ]]; then
            index=$((10#${BASH_REMATCH[1]}))
            (( index > maximum_index )) && maximum_index=${index}
        fi
    done

    printf '%s\n' "$((maximum_index + 1))"
}

next_available_episode() {
    local candidate="$1"
    while ! episode_is_free "${candidate}"; do
        printf 'Skipping occupied episode_%s.\n' "${candidate}" >&2
        candidate=$((candidate + 1))
    done
    printf '%s\n' "${candidate}"
}

prompt_initial_episode() {
    local suggested="$1"
    local input
    local candidate

    while true; do
        printf '\nAutomatically suggested first ID: episode_%s\n' "${suggested}"
        printf 'Press Enter to accept, type another non-negative ID, or type Q to quit: '
        IFS= read -r input || exit 0
        if [[ "${input,,}" == "q" ]]; then
            exit 0
        elif [[ -z "${input}" ]]; then
            candidate="${suggested}"
        elif [[ "${input}" =~ ^[0-9]+$ ]]; then
            candidate=$((10#${input}))
        else
            printf 'Please enter a non-negative integer, Enter, or Q.\n' >&2
            continue
        fi

        if episode_is_free "${candidate}"; then
            START_EPISODE="${candidate}"
            return
        fi
        printf 'episode_%s already has HDF5 or export data; choose another starting ID.\n' \
            "${candidate}" >&2
    done
}

prompt_episode_start() {
    local episode_index="$1"
    local input
    printf '\nReady for episode_%s. Press Enter when the operator and scene are ready, or Q to quit: ' \
        "${episode_index}"
    IFS= read -r input || return 1
    [[ "${input,,}" != "q" ]]
}

collect_and_export() {
    local episode_index="$1"
    local episode_stem="episode_${episode_index}"
    local hdf5_path="${DATASET_DIR}/${TASK_NAME}/${episode_stem}.hdf5"
    local episode_export_dir="${EXPORT_DIR}/${TASK_NAME}/${episode_stem}"

    [[ ! -e "${hdf5_path}" ]] || {
        printf 'Error: HDF5 already exists: %s\n' "${hdf5_path}" >&2
        return 1
    }
    [[ ! -e "${episode_export_dir}" ]] || {
        printf 'Error: export directory already exists: %s\n' "${episode_export_dir}" >&2
        return 1
    }

    printf '\nStarting %s: maximum %s timesteps at %s Hz. Press S to stop and save.\n' \
        "${episode_stem}" "${MAX_TIMESTEPS}" "${FRAME_RATE}"
    speak "Episode ${episode_index} collection started"

    if ! "${COLLECT_PYTHON}" "${COLLECT_SCRIPT}" \
        --dataset-dir "${DATASET_DIR}" \
        --task-name "${TASK_NAME}" \
        --episode-idx "${episode_index}" \
        --max-timesteps "${MAX_TIMESTEPS}" \
        --frame-rate "${FRAME_RATE}" \
        --sync-timeout "${SYNC_TIMEOUT}"; then
        speak "Episode ${episode_index} collection failed and ended"
        return 1
    fi

    [[ -s "${hdf5_path}" ]] || {
        printf 'Error: collection succeeded but HDF5 is missing: %s\n' "${hdf5_path}" >&2
        return 1
    }
    speak "Episode ${episode_index} collection ended"

    printf 'Exporting videos and diagnostics for %s.\n' "${episode_stem}"
    if ! PYTHONNOUSERSITE=1 MPLCONFIGDIR="${MPL_CONFIG_DIR}" "${EXPORT_PYTHON}" \
        "${EXPORT_SCRIPT}" \
        "${hdf5_path}" \
        --output-dir "${episode_export_dir}" \
        --fps "${FRAME_RATE}" \
        --source-color rgb; then
        printf 'Error: export failed for %s; HDF5 is preserved.\n' "${episode_stem}" >&2
        return 1
    fi

    if ! check_export_outputs "${episode_export_dir}" "${episode_stem}"; then
        printf 'Error: one or more export outputs are missing for %s.\n' "${episode_stem}" >&2
        return 1
    fi
    printf 'Completed %s.\n' "${episode_stem}"
}

replace_current_episode() {
    local episode_index="$1"
    local episode_stem="episode_${episode_index}"
    local hdf5_path="${DATASET_DIR}/${TASK_NAME}/${episode_stem}.hdf5"
    local episode_export_dir="${EXPORT_DIR}/${TASK_NAME}/${episode_stem}"
    local backup_suffix=".interactive_backup_${BASHPID}"
    local backup_hdf5="${hdf5_path}${backup_suffix}"
    local backup_export_dir="${episode_export_dir}${backup_suffix}"

    [[ -f "${hdf5_path}" ]] || {
        printf 'Error: current HDF5 is missing: %s\n' "${hdf5_path}" >&2
        return 1
    }
    [[ -d "${episode_export_dir}" ]] || {
        printf 'Error: current export directory is missing: %s\n' "${episode_export_dir}" >&2
        return 1
    }
    [[ ! -e "${backup_hdf5}" && ! -e "${backup_export_dir}" ]] || {
        printf 'Error: retry backup already exists for %s.\n' "${episode_stem}" >&2
        return 1
    }

    mv -- "${hdf5_path}" "${backup_hdf5}" || return 1
    if ! mv -- "${episode_export_dir}" "${backup_export_dir}"; then
        mv -- "${backup_hdf5}" "${hdf5_path}"
        return 1
    fi

    printf 'The previous %s is backed up while recollection runs.\n' "${episode_stem}"
    if collect_and_export "${episode_index}"; then
        rm -f -- "${backup_hdf5}"
        rm -rf -- "${backup_export_dir}"
        printf 'Replaced %s successfully.\n' "${episode_stem}"
        return 0
    fi

    printf 'Recollection failed; restoring the previous %s.\n' "${episode_stem}" >&2
    rm -f -- "${hdf5_path}"
    rm -rf -- "${episode_export_dir}"
    mv -- "${backup_hdf5}" "${hdf5_path}"
    mv -- "${backup_export_dir}" "${episode_export_dir}"
    return 1
}

while (( $# > 0 )); do
    case "$1" in
        --start-episode)
            require_option_value "$1" "$#"
            START_EPISODE="$2"
            shift 2
            ;;
        --dataset-dir)
            require_option_value "$1" "$#"
            DATASET_DIR="$2"
            shift 2
            ;;
        --export-dir)
            require_option_value "$1" "$#"
            EXPORT_DIR="$2"
            shift 2
            ;;
        --task-name)
            require_option_value "$1" "$#"
            TASK_NAME="$2"
            shift 2
            ;;
        --max-timesteps)
            require_option_value "$1" "$#"
            MAX_TIMESTEPS="$2"
            shift 2
            ;;
        --frame-rate)
            require_option_value "$1" "$#"
            FRAME_RATE="$2"
            shift 2
            ;;
        --sync-timeout)
            require_option_value "$1" "$#"
            SYNC_TIMEOUT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

[[ "${MAX_TIMESTEPS}" =~ ^[1-9][0-9]*$ ]] || die "--max-timesteps must be a positive integer"
[[ "${FRAME_RATE}" =~ ^[1-9][0-9]*$ ]] || die "--frame-rate must be a positive integer"
[[ "${SYNC_TIMEOUT}" =~ ^[1-9][0-9]*$ ]] || die "--sync-timeout must be a positive integer"
if [[ -n "${START_EPISODE}" ]]; then
    [[ "${START_EPISODE}" =~ ^[0-9]+$ ]] || die "--start-episode must be a non-negative integer"
    START_EPISODE=$((10#${START_EPISODE}))
fi
[[ -n "${TASK_NAME}" ]] || die "--task-name cannot be empty"
[[ "${TASK_NAME}" != */* && "${TASK_NAME}" != "." && "${TASK_NAME}" != ".." ]] || \
    die "--task-name must be a single directory name"

DATASET_DIR="${DATASET_DIR/#\~/${HOME}}"
EXPORT_DIR="${EXPORT_DIR/#\~/${HOME}}"

[[ -r "${ROS_SETUP}" ]] || die "ROS 2 setup not found: ${ROS_SETUP}"
[[ -r "${WORKSPACE_SETUP}" ]] || die "workspace setup not found: ${WORKSPACE_SETUP}"
[[ -r "${COLLECT_SCRIPT}" ]] || die "interactive collector not found: ${COLLECT_SCRIPT}"
[[ -x "${COLLECT_PYTHON}" ]] || die "collection Python not found: ${COLLECT_PYTHON}"
[[ -r "${EXPORT_SCRIPT}" ]] || die "export script not found: ${EXPORT_SCRIPT}"
[[ -x "${EXPORT_PYTHON}" ]] || die "export Python not found: ${EXPORT_PYTHON}"
[[ -t 0 ]] || die "this script requires an interactive terminal"

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export ROS_LOCALHOST_ONLY=1

command -v ros2 >/dev/null 2>&1 || die "ros2 is unavailable after sourcing the environment"
ros2 pkg prefix cobot_magic_ros2 >/dev/null || die "ROS 2 package cobot_magic_ros2 is unavailable"
mkdir -p "${MPL_CONFIG_DIR}"
PYTHONNOUSERSITE=1 MPLCONFIGDIR="${MPL_CONFIG_DIR}" "${EXPORT_PYTHON}" -c \
    'import cv2, h5py, matplotlib, numpy' || die "the export Python environment is incomplete"
"${COLLECT_PYTHON}" -c 'import h5py, numpy, rclpy; import cobot_magic_ros2.collect_data' || \
    die "the ROS 2 collection Python environment is incomplete"

mkdir -p "${DATASET_DIR}/${TASK_NAME}" "${EXPORT_DIR}/${TASK_NAME}"
check_required_topics

if [[ -n "${START_EPISODE}" ]]; then
    suggested_episode="${START_EPISODE}"
else
    suggested_episode="$(detect_start_episode)"
fi
prompt_initial_episode "${suggested_episode}"

printf '\nInteractive collection configuration:\n'
printf '  first episode: episode_%s\n' "${START_EPISODE}"
printf '  HDF5 root:     %s/%s\n' "${DATASET_DIR}" "${TASK_NAME}"
printf '  export root:   %s/%s\n' "${EXPORT_DIR}" "${TASK_NAME}"
printf '  maximum:       %s timesteps at %s Hz\n' "${MAX_TIMESTEPS}" "${FRAME_RATE}"
printf '  stop/save key: S\n'

episode_index="${START_EPISODE}"
while true; do
    if ! prompt_episode_start "${episode_index}"; then
        printf '\nStopped before episode_%s.\n' "${episode_index}"
        exit 0
    fi

    if ! collect_and_export "${episode_index}"; then
        die "collection or export failed for episode_${episode_index}"
    fi

    while true; do
        printf '\nChoose the next action:\n'
        printf '  C - continue with the next episode ID\n'
        printf '  R - recollect and replace episode_%s\n' "${episode_index}"
        printf '  Q - stop\n'
        printf 'Selection [C/R/Q]: '
        IFS= read -r choice || choice="q"
        case "${choice,,}" in
            c)
                episode_index="$(next_available_episode "$((episode_index + 1))")"
                break
                ;;
            r)
                printf 'Recollecting will replace only episode_%s after the retry succeeds.\n' \
                    "${episode_index}"
                if ! prompt_episode_start "${episode_index}"; then
                    printf '\nRecollection cancelled; episode_%s remains unchanged.\n' "${episode_index}"
                    continue
                fi
                replace_current_episode "${episode_index}" || true
                ;;
            q)
                printf '\nStopped after episode_%s.\n' "${episode_index}"
                exit 0
                ;;
            *)
                printf 'Please enter C, R, or Q.\n' >&2
                ;;
        esac
    done
done
