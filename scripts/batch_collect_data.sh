#!/usr/bin/env bash
set -Eeuo pipefail

readonly FRAME_RATE=20
readonly MAX_TIMESTEPS=600
readonly INTERVAL_SECONDS=25

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
EXPORT_SCRIPT="${EXPORT_SCRIPT:-${WORKSPACE_ROOT}/tools/export_hdf5_episode.py}"
EXPORT_PYTHON="${EXPORT_PYTHON:-python3}"
MPL_CONFIG_DIR="${TMPDIR:-/tmp}/cobot_magic_mplconfig"

EPISODE_COUNT=""
START_EPISODE=""
DATASET_DIR="${HOME}/data"
EXPORT_DIR="${HOME}/data_mp4"
TASK_NAME="DLO_Stretch"

usage() {
    cat <<'EOF'
Usage:
  batch_collect_data.sh --episodes COUNT [options]

Required:
  --episodes COUNT       Number of episodes to collect.

Options:
  --start-episode INDEX  Override the automatically detected first index.
  --dataset-dir PATH     HDF5 root directory (default: ~/data).
  --export-dir PATH      Export root directory (default: ~/data_mp4).
  --task-name NAME       Task subdirectory name (default: DLO_Stretch).
  -h, --help             Show this help message.

Fixed collection settings:
  frame rate: 20 Hz
  timesteps:  600 per episode
  interval:   25 seconds after collection finishes
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
    spd-say --wait --language en "$message"
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
        [[ -s "${episode_dir}/${output}" ]] || die "missing or empty export: ${episode_dir}/${output}"
    done
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

while (( $# > 0 )); do
    case "$1" in
        --episodes)
            require_option_value "$1" "$#"
            EPISODE_COUNT="$2"
            shift 2
            ;;
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
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

[[ "${EPISODE_COUNT}" =~ ^[1-9][0-9]*$ ]] || die "--episodes must be a positive integer"
if [[ -n "${START_EPISODE}" ]]; then
    [[ "${START_EPISODE}" =~ ^[0-9]+$ ]] || die "--start-episode must be a non-negative integer"
    START_EPISODE=$((10#${START_EPISODE}))
fi
[[ -n "${TASK_NAME}" ]] || die "--task-name cannot be empty"
[[ "${TASK_NAME}" != */* && "${TASK_NAME}" != "." && "${TASK_NAME}" != ".." ]] || \
    die "--task-name must be a single directory name"

[[ -r "${ROS_SETUP}" ]] || die "ROS 2 setup not found: ${ROS_SETUP}"
[[ -r "${WORKSPACE_SETUP}" ]] || die "workspace setup not found: ${WORKSPACE_SETUP}"
[[ -r "${EXPORT_SCRIPT}" ]] || die "export script not found: ${EXPORT_SCRIPT}"
if ! EXPORT_PYTHON="$(command -v "${EXPORT_PYTHON}" 2>/dev/null)"; then
    [[ -x "${EXPORT_PYTHON}" ]] || die "export Python not found: ${EXPORT_PYTHON}"
fi
command -v spd-say >/dev/null 2>&1 || die "spd-say is required for voice prompts"

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export ROS_LOCALHOST_ONLY=1

command -v ros2 >/dev/null 2>&1 || die "ros2 command is unavailable after sourcing the environment"
ros2 pkg prefix cobot_magic_ros2 >/dev/null || die "ROS 2 package cobot_magic_ros2 is unavailable"
mkdir -p "${MPL_CONFIG_DIR}"
MPLCONFIGDIR="${MPL_CONFIG_DIR}" "${EXPORT_PYTHON}" -c \
    'import cv2, h5py, matplotlib, numpy' || die "the export Python environment is incomplete"

mkdir -p "${DATASET_DIR}/${TASK_NAME}" "${EXPORT_DIR}/${TASK_NAME}"

if [[ -z "${START_EPISODE}" ]]; then
    START_EPISODE="$(detect_start_episode)"
    printf 'Automatically selected episode_%s as the first episode.\n' "${START_EPISODE}"
else
    printf 'Using manually selected episode_%s as the first episode.\n' "${START_EPISODE}"
fi

# Check the whole requested range before collecting anything. collect_data writes with
# mode "w", so this guard is what prevents accidental replacement of an old episode.
for (( offset = 0; offset < EPISODE_COUNT; offset++ )); do
    episode_index=$((START_EPISODE + offset))
    episode_stem="episode_${episode_index}"
    hdf5_path="${DATASET_DIR}/${TASK_NAME}/${episode_stem}.hdf5"
    episode_export_dir="${EXPORT_DIR}/${TASK_NAME}/${episode_stem}"
    [[ ! -e "${hdf5_path}" ]] || die "HDF5 already exists: ${hdf5_path}"
    [[ ! -e "${episode_export_dir}" ]] || die "export directory already exists: ${episode_export_dir}"
done

check_required_topics

printf 'Batch collection configuration:\n'
printf '  episodes:      %s\n' "${EPISODE_COUNT}"
printf '  start episode: %s\n' "${START_EPISODE}"
printf '  HDF5 root:     %s/%s\n' "${DATASET_DIR}" "${TASK_NAME}"
printf '  export root:   %s/%s\n' "${EXPORT_DIR}" "${TASK_NAME}"
printf '  sampling:      %s steps at %s Hz\n' "${MAX_TIMESTEPS}" "${FRAME_RATE}"
printf '  interval:      %s seconds\n' "${INTERVAL_SECONDS}"

for (( offset = 0; offset < EPISODE_COUNT; offset++ )); do
    episode_index=$((START_EPISODE + offset))
    episode_stem="episode_${episode_index}"
    hdf5_path="${DATASET_DIR}/${TASK_NAME}/${episode_stem}.hdf5"
    episode_export_dir="${EXPORT_DIR}/${TASK_NAME}/${episode_stem}"

    printf '\n[%s/%s] Starting %s\n' "$((offset + 1))" "${EPISODE_COUNT}" "${episode_stem}"
    speak "Episode ${episode_index} collection started"

    if ! ros2 run cobot_magic_ros2 collect_data -- \
        --dataset_dir "${DATASET_DIR}" \
        --task_name "${TASK_NAME}" \
        --episode_idx "${episode_index}" \
        --max_timesteps "${MAX_TIMESTEPS}" \
        --frame_rate "${FRAME_RATE}"; then
        speak "Episode ${episode_index} collection failed and ended" || true
        die "collection failed for ${episode_stem}"
    fi

    collection_finished_at=$(date +%s)
    [[ -s "${hdf5_path}" ]] || die "collection returned successfully but HDF5 is missing: ${hdf5_path}"
    speak "Episode ${episode_index} collection ended"

    printf 'Exporting videos and diagnostics for %s\n' "${episode_stem}"
    if ! MPLCONFIGDIR="${MPL_CONFIG_DIR}" "${EXPORT_PYTHON}" \
        "${EXPORT_SCRIPT}" \
        "${hdf5_path}" \
        --output-dir "${episode_export_dir}" \
        --fps "${FRAME_RATE}" \
        --source-color rgb; then
        die "export failed for ${episode_stem}; HDF5 is preserved at ${hdf5_path}"
    fi
    check_export_outputs "${episode_export_dir}" "${episode_stem}"
    printf 'Completed %s\n' "${episode_stem}"

    if (( offset + 1 < EPISODE_COUNT )); then
        elapsed_since_collection=$(( $(date +%s) - collection_finished_at ))
        remaining_interval=$((INTERVAL_SECONDS - elapsed_since_collection))
        if (( remaining_interval > 0 )); then
            printf 'Waiting %s seconds before the next episode\n' "${remaining_interval}"
            sleep "${remaining_interval}"
        else
            printf 'Export took %s seconds; starting the next episode without additional delay\n' \
                "${elapsed_since_collection}"
        fi
    fi
done

printf '\nAll %s episodes completed successfully.\n' "${EPISODE_COUNT}"
