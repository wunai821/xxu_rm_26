#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: record_mid360_regression.sh LABEL [options]

Record the raw inputs and outputs needed to reproduce a rotating-gimbal LIO run.

Options:
  --duration SEC          Stop after SEC seconds (default: record until Ctrl-C)
  --output-root DIR       Parent directory (default: ./bags/lio_regression)
  --lidar-topic TOPIC     Raw MID360 PointCloud2 (default: /livox/lidar)
  --imu-topic TOPIC       Raw MID360 IMU (default: /livox/imu)
  --joint-topic TOPIC     Gimbal JointState (default: /joint_states)
  --lio-odom-topic TOPIC  Small Point-LIO output (default: /odom)
  --reference-odom TOPIC  Independent wheel/mocap odometry to include
  --include-derived-clouds  Also record deskewed/compensated point clouds
  --storage-profile NAME  MCAP profile: fastwrite, zstd_fast or zstd_small
                          (default: fastwrite, to minimize recorder CPU load)
  -h, --help              Show this help

The ROS 2 environment and the robot workspace must already be sourced.
EOF
}

if [[ $# -gt 0 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
  usage
  exit 0
fi
if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

label=$1
shift
duration=""
output_root="${PWD}/bags/lio_regression"
lidar_topic="/livox/lidar"
imu_topic="/livox/imu"
joint_topic="/joint_states"
lio_odom_topic="/odom"
reference_odom=""
include_derived_clouds=false
storage_profile="fastwrite"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      duration=${2:?missing value for --duration}
      shift 2
      ;;
    --output-root)
      output_root=${2:?missing value for --output-root}
      shift 2
      ;;
    --lidar-topic)
      lidar_topic=${2:?missing value for --lidar-topic}
      shift 2
      ;;
    --imu-topic)
      imu_topic=${2:?missing value for --imu-topic}
      shift 2
      ;;
    --joint-topic)
      joint_topic=${2:?missing value for --joint-topic}
      shift 2
      ;;
    --lio-odom-topic)
      lio_odom_topic=${2:?missing value for --lio-odom-topic}
      shift 2
      ;;
    --reference-odom)
      reference_odom=${2:?missing value for --reference-odom}
      shift 2
      ;;
    --include-derived-clouds)
      include_derived_clouds=true
      shift
      ;;
    --storage-profile)
      storage_profile=${2:?missing value for --storage-profile}
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not available; source ROS 2 and the workspace first." >&2
  exit 1
fi
if [[ -n "$duration" ]] && {
  ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    [[ -z "${duration//[.0]/}" ]]
}; then
  echo "--duration must be a positive number of seconds." >&2
  exit 2
fi
case "$storage_profile" in
  fastwrite|zstd_fast|zstd_small) ;;
  *)
    echo "--storage-profile must be fastwrite, zstd_fast or zstd_small." >&2
    exit 2
    ;;
esac
if [[ -n "$reference_odom" && "$reference_odom" == "$lio_odom_topic" ]]; then
  echo "--reference-odom must differ from --lio-odom-topic." >&2
  exit 2
fi

safe_label=$(printf '%s' "$label" | tr -cs '[:alnum:]_-' '_')
if [[ -z "$safe_label" ]]; then
  safe_label="run"
fi
timestamp=$(date +%Y%m%d_%H%M%S)
mkdir -p "$output_root"
bag_path="${output_root}/${timestamp}_${safe_label}"

required_topics=("$lidar_topic" "$imu_topic" "$joint_topic" "$lio_odom_topic")
if [[ -n "$reference_odom" ]]; then
  required_topics+=("$reference_odom")
fi
missing_topics=()
invalid_types=()
declare -A expected_types
expected_types["$lidar_topic"]="sensor_msgs/msg/PointCloud2"
expected_types["$imu_topic"]="sensor_msgs/msg/Imu"
expected_types["$joint_topic"]="sensor_msgs/msg/JointState"
expected_types["$lio_odom_topic"]="nav_msgs/msg/Odometry"
if [[ -n "$reference_odom" ]]; then
  expected_types["$reference_odom"]="nav_msgs/msg/Odometry"
fi
for topic in "${required_topics[@]}"; do
  actual_type=$(ros2 topic type "$topic" 2>/dev/null || true)
  if [[ -z "$actual_type" ]]; then
    missing_topics+=("$topic")
  elif [[ "$actual_type" != "${expected_types[$topic]}" ]]; then
    invalid_types+=("$topic (expected ${expected_types[$topic]}, got $actual_type)")
  fi
done
if [[ ${#missing_topics[@]} -gt 0 ]]; then
  echo "Required topics are not currently published:" >&2
  printf '  %s\n' "${missing_topics[@]}" >&2
  echo "Start the MID360, joint-state publisher and Small Point-LIO before recording." >&2
  exit 1
fi
if [[ ${#invalid_types[@]} -gt 0 ]]; then
  echo "Required topics have incompatible types:" >&2
  printf '  %s\n' "${invalid_types[@]}" >&2
  exit 1
fi

topics=("$lidar_topic" "$imu_topic" "$joint_topic" "$lio_odom_topic")
optional_topics=(
  /tf
  /tf_static
  /imu_lio_compensated
  /cmd_vel
  /cmd_vel_keyboard
  /cmd_vel_transformed
)
if [[ "$include_derived_clouds" == true ]]; then
  optional_topics+=(
    /cloud_deskewed
    /livox/lidar_compensated
    /mid360/livox_points_compensated
  )
fi
if [[ -n "$reference_odom" ]]; then
  optional_topics+=("$reference_odom")
fi

for topic in "${optional_topics[@]}"; do
  if ros2 topic type "$topic" >/dev/null 2>&1; then
    already_added=false
    for existing in "${topics[@]}"; do
      if [[ "$existing" == "$topic" ]]; then
        already_added=true
        break
      fi
    done
    if [[ "$already_added" == false ]]; then
      topics+=("$topic")
    fi
  fi
done

manifest_tmp=$(mktemp "${output_root}/.lio_manifest.XXXXXX")
cleanup() {
  rm -f "$manifest_tmp"
}
trap cleanup EXIT
{
  if git_commit=$(git rev-parse HEAD 2>/dev/null); then
    git_dirty=$(git status --porcelain 2>/dev/null | wc -l)
  else
    git_commit="unknown"
    git_dirty="unknown"
  fi
  echo "label=$label"
  echo "recorded_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "lidar_topic=$lidar_topic"
  echo "imu_topic=$imu_topic"
  echo "joint_topic=$joint_topic"
  echo "lio_odom_topic=$lio_odom_topic"
  echo "reference_odom=$reference_odom"
  echo "git_commit=$git_commit"
  echo "git_dirty=$git_dirty"
  echo "storage_profile=$storage_profile"
  echo "include_derived_clouds=$include_derived_clouds"
  printf 'topics=%s\n' "${topics[*]}"
} >"$manifest_tmp"

echo "Recording: $bag_path"
printf '  %s\n' "${topics[@]}"

record_command=(
  ros2 bag record
  --storage mcap
  --storage-preset-profile "$storage_profile"
  --output "$bag_path"
  --custom-data "test_label=$safe_label"
  --disable-keyboard-controls
  --topics "${topics[@]}"
)

status=0
if [[ -n "$duration" ]]; then
  set +e
  timeout --signal=INT --kill-after=10 "${duration}s" "${record_command[@]}"
  status=$?
  set -e
  if [[ $status -eq 124 ]]; then
    status=0
  fi
else
  set +e
  "${record_command[@]}"
  status=$?
  set -e
fi

if [[ -d "$bag_path" ]]; then
  mv "$manifest_tmp" "$bag_path/capture_manifest.txt"
  ros2 topic list -t >"$bag_path/ros_topics_at_end.txt" 2>/dev/null || true
  if ros2 node list 2>/dev/null | rg -q '^/small_point_lio$'; then
    ros2 param dump /small_point_lio >"$bag_path/small_point_lio_params.yaml" 2>/dev/null || true
  fi
  echo "Saved: $bag_path"
fi

exit "$status"
