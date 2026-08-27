#!/bin/bash

# ================================================================
# UGV02 2-PASS AUTONOMOUS SYSTEM
#
# STAGE 1
#   Wall Follow
#   + Cartographer SLAM
#   + YOLO Safety
#   + Automatic Lap Detection
#
#        ↓
#
#   save_mapping_result.py
#
#        ↓
#
#   factory_map.pbstream
#   patrol_path.csv
#
#        ↓
#
# STAGE 2
#   Cartographer Pure Localization
#   + Pure Pursuit Path Following
#   + YOLO Safety
# ================================================================


# ================================================================
# ROS Environment
# ================================================================

source /opt/ros/humble/setup.bash
source /home/mice/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/mice/cyclonedds.xml
export ROS_DOMAIN_ID=0


# ================================================================
# Paths
# ================================================================

BASE_DIR="/home/mice/ugv02_slam_test"

MAP_DIR="$BASE_DIR/maps"

MAP_FILE="$MAP_DIR/factory_map.pbstream"

PATH_FILE="$MAP_DIR/patrol_path.csv"

MAPPING_LUA="$BASE_DIR/cartographer_a1.lua"

LOCALIZATION_LUA="$BASE_DIR/cartographer_localization.lua"

WALL_NODE="$BASE_DIR/wall_follow_node.py"

PATH_NODE="$BASE_DIR/path_follow_node.py"

BRIDGE_NODE="$BASE_DIR/ugv02_serial_bridge.py"

YOLO_NODE="$BASE_DIR/yolo_safety_node.py"

YOLO_PYTHON="/home/mice/yolo_practice/.venv/bin/python"


# ================================================================
# Stable USB Device Paths
# ================================================================

LIDAR_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

ESP_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C37186196-if00"


# ================================================================
# Logs
# ================================================================

LOG_DIR="$BASE_DIR/two_pass_logs"

mkdir -p "$LOG_DIR"

mkdir -p "$MAP_DIR"


# ================================================================
# Process IDs
# ================================================================

LIDAR_PID=""

BRIDGE_PID=""

YOLO_PID=""

TF_PID=""

CARTO_PID=""

GRID_PID=""

DRIVE_PID=""

MAP_DONE_WATCH_PID=""


# ================================================================
# Utility
# ================================================================

kill_process_group()
{
    PID="$1"

    if [ -n "$PID" ]
    then

        if kill -0 "$PID" 2>/dev/null
        then

            kill -TERM -- "-$PID" \
                2>/dev/null || true

        fi

    fi
}


# ================================================================
# Cleanup
# ================================================================

cleanup()
{
    echo
    echo "=========================================="
    echo "STOPPING UGV02 2-PASS SYSTEM"
    echo "=========================================="

    # ------------------------------------------------------------
    # 주행 노드를 가장 먼저 종료
    # ------------------------------------------------------------

    kill_process_group "$DRIVE_PID"

    DRIVE_PID=""

    sleep 1


    # Serial Bridge에는 cmd_vel timeout이 있으므로
    # 주행 노드 종료 후 잠시 기다려 0 명령 전송 기회 제공

    kill_process_group "$MAP_DONE_WATCH_PID"

    kill_process_group "$GRID_PID"

    kill_process_group "$CARTO_PID"

    kill_process_group "$YOLO_PID"

    kill_process_group "$TF_PID"

    kill_process_group "$BRIDGE_PID"

    kill_process_group "$LIDAR_PID"


    wait 2>/dev/null || true


    echo
    echo "UGV02 SYSTEM SAFELY STOPPED."
    echo "=========================================="
}


trap cleanup EXIT

trap 'exit 0' INT TERM


# ================================================================
# Required Files Check
# ================================================================

echo
echo "=========================================="
echo "UGV02 2-PASS AUTONOMOUS SYSTEM"
echo "=========================================="


for FILE in \
    "$MAPPING_LUA" \
    "$LOCALIZATION_LUA" \
    "$WALL_NODE" \
    "$PATH_NODE" \
    "$BRIDGE_NODE" \
    "$YOLO_NODE"

do

    if [ ! -f "$FILE" ]
    then

        echo
        echo "ERROR:"
        echo "Required file not found:"
        echo "$FILE"

        exit 1

    fi

done


# ================================================================
# Backup Previous Map / Path
# ================================================================

BACKUP_DIR="$MAP_DIR/archive"

mkdir -p "$BACKUP_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)


if [ -f "$MAP_FILE" ]
then

    cp "$MAP_FILE" \
       "$BACKUP_DIR/factory_map_${TIMESTAMP}.pbstream"

fi


if [ -f "$PATH_FILE" ]
then

    cp "$PATH_FILE" \
       "$BACKUP_DIR/patrol_path_${TIMESTAMP}.csv"

fi


echo
echo "Previous map/path backup complete."


# ================================================================
# [1] Wait for USB Devices
# ================================================================

echo
echo "=========================================="
echo "[1] WAITING FOR HARDWARE"
echo "=========================================="


echo
echo "Waiting for RPLIDAR..."


while [ ! -e "$LIDAR_PORT" ]
do

    sleep 1

done


echo "RPLIDAR detected:"
echo "$LIDAR_PORT"


echo
echo "Waiting for ESP32..."


while [ ! -e "$ESP_PORT" ]
do

    sleep 1

done


echo "ESP32 detected:"
echo "$ESP_PORT"


# ================================================================
# [2] Static TF
# ================================================================

echo
echo "=========================================="
echo "[2] STARTING STATIC TF"
echo "=========================================="


setsid ros2 run \
    tf2_ros \
    static_transform_publisher \
    --x 0 \
    --y 0 \
    --z 0 \
    --roll 0 \
    --pitch 0 \
    --yaw 0 \
    --frame-id base_link \
    --child-frame-id laser_link \
    > "$LOG_DIR/tf.log" 2>&1 &


TF_PID=$!


sleep 2


if ! kill -0 "$TF_PID" 2>/dev/null
then

    echo "ERROR: Static TF failed."

    cat "$LOG_DIR/tf.log"

    exit 1

fi


echo "Static TF READY"


# ================================================================
# [3] RPLIDAR
# ================================================================

echo
echo "=========================================="
echo "[3] STARTING RPLIDAR"
echo "=========================================="


setsid ros2 launch \
    sllidar_ros2 \
    sllidar_a1_launch.py \
    serial_port:="$LIDAR_PORT" \
    serial_baudrate:=115200 \
    frame_id:=laser_link \
    > "$LOG_DIR/lidar.log" 2>&1 &


LIDAR_PID=$!


sleep 3


if ! kill -0 "$LIDAR_PID" 2>/dev/null
then

    echo
    echo "ERROR: RPLIDAR process terminated."

    cat "$LOG_DIR/lidar.log"

    exit 1

fi


echo "Waiting for /scan..."


if timeout 20 \
    ros2 topic echo \
    /scan \
    sensor_msgs/msg/LaserScan \
    --qos-reliability best_effort \
    --once \
    > /dev/null 2>&1
then

    echo "/scan READY"

else

    echo
    echo "ERROR: /scan not received."

    cat "$LOG_DIR/lidar.log"

    exit 1

fi


# ================================================================
# [4] Serial Bridge
# ================================================================

echo
echo "=========================================="
echo "[4] STARTING ESP32 SERIAL BRIDGE"
echo "=========================================="


setsid python3 \
    "$BRIDGE_NODE" \
    --ros-args \
    -p port:="$ESP_PORT" \
    -p baudrate:=115200 \
    > "$LOG_DIR/bridge.log" 2>&1 &


BRIDGE_PID=$!


sleep 3


if ! kill -0 "$BRIDGE_PID" 2>/dev/null
then

    echo
    echo "ERROR: Serial Bridge terminated."

    cat "$LOG_DIR/bridge.log"

    exit 1

fi


echo "Serial Bridge READY"


# ================================================================
# [5] YOLO Safety
# ================================================================

echo
echo "=========================================="
echo "[5] STARTING YOLO SAFETY"
echo "=========================================="


setsid "$YOLO_PYTHON" \
    "$YOLO_NODE" \
    > "$LOG_DIR/yolo.log" 2>&1 &


YOLO_PID=$!


echo "Waiting for cameras and YOLO..."


VISION_READY=0


for i in $(seq 1 60)
do

    if ! kill -0 "$YOLO_PID" 2>/dev/null
    then

        echo
        echo "ERROR: YOLO process terminated."

        cat "$LOG_DIR/yolo.log"

        exit 1

    fi


    RESULT=$(

        timeout 5 \
        ros2 topic echo \
        /vision_ready \
        std_msgs/msg/Bool \
        --once \
        2>/dev/null || true

    )


    if echo "$RESULT" \
        | grep -q "data: true"
    then

        VISION_READY=1

        break

    fi


    sleep 1

done


if [ "$VISION_READY" -ne 1 ]
then

    echo
    echo "ERROR: YOLO Safety did not become READY."

    cat "$LOG_DIR/yolo.log"

    exit 1

fi


echo "YOLO Safety READY"


# ================================================================
# [6] Cartographer Mapping
# ================================================================

echo
echo "=========================================="
echo "[6] STARTING CARTOGRAPHER MAPPING"
echo "=========================================="


setsid ros2 run \
    cartographer_ros \
    cartographer_node \
    -configuration_directory "$BASE_DIR" \
    -configuration_basename cartographer_a1.lua \
    > "$LOG_DIR/cartographer_mapping.log" 2>&1 &


CARTO_PID=$!


sleep 5


if ! kill -0 "$CARTO_PID" 2>/dev/null
then

    echo
    echo "ERROR: Cartographer Mapping terminated."

    cat "$LOG_DIR/cartographer_mapping.log"

    exit 1

fi


echo "Cartographer Mapping READY"


# ================================================================
# [7] Occupancy Grid
# ================================================================

echo
echo "=========================================="
echo "[7] STARTING OCCUPANCY GRID"
echo "=========================================="


setsid ros2 run \
    cartographer_ros \
    cartographer_occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 \
    > "$LOG_DIR/grid_mapping.log" 2>&1 &


GRID_PID=$!


sleep 4


if ! kill -0 "$GRID_PID" 2>/dev/null
then

    echo
    echo "ERROR: Occupancy Grid terminated."

    cat "$LOG_DIR/grid_mapping.log"

    exit 1

fi


echo "Waiting for /map..."


MAP_READY=0


for i in $(seq 1 20)
do

    if timeout 10 \
        ros2 topic echo \
        /map \
        nav_msgs/msg/OccupancyGrid \
        --once \
        > /dev/null 2>&1
    then

        MAP_READY=1

        break

    fi


    sleep 1

done


if [ "$MAP_READY" -ne 1 ]
then

    echo
    echo "ERROR: /map was not received."

    exit 1

fi


echo "/map READY"


# ================================================================
# STAGE 1
# ================================================================

echo
echo
echo "################################################"
echo "#                                              #"
echo "#           STAGE 1 : MAP-RUN                 #"
echo "#                                              #"
echo "################################################"
echo
echo "Wall Follow + SLAM"
echo "Automatic Lap Detection"
echo "Automatic Map/Path Saving"
echo


# ================================================================
# Map-Run Completion Watcher
#
# wall_follow_node가 완료 메시지를 단 한 번만 publish하므로
# 주행 노드를 실행하기 전에 subscriber를 먼저 실행
# ================================================================

MAP_DONE_LOG="$LOG_DIR/map_run_finished.log"

rm -f "$MAP_DONE_LOG"


setsid ros2 topic echo \
    /map_run_finished \
    std_msgs/msg/Bool \
    --once \
    > "$MAP_DONE_LOG" 2>&1 &


MAP_DONE_WATCH_PID=$!


sleep 1


# ================================================================
# Start Wall Follow
#
# STAGE 1 완료 조건
#
# 1. 시작점에서 1.00 m 이상 벗어난 적이 있어야 함
# 2. 실제 누적 주행거리 3.00 m 이상
# 3. 주행시간 15초 이상
# 4. 시작점 반경 0.10 m (10 cm) 안으로 복귀
# ================================================================

setsid python3 \
    "$WALL_NODE" \
    --ros-args \
    -p start_enabled:=true \
    -p leave_start_radius:=1.00 \
    -p return_radius:=0.10 \
    -p min_lap_distance:=3.00 \
    -p min_lap_time:=15.0 \
    > "$LOG_DIR/wall_follow.log" 2>&1 &


DRIVE_PID=$!


echo
echo "STAGE 1 STARTED"
echo
echo "Robot is now performing:"
echo "  Wall Following"
echo "  SLAM Mapping"
echo "  Lap Detection"
echo


# ================================================================
# Wait until wall_follow_node announces saved map/path
# ================================================================

while true
do

    if ! kill -0 "$DRIVE_PID" 2>/dev/null
    then

        echo
        echo "ERROR:"
        echo "wall_follow_node terminated before lap completion."

        cat "$LOG_DIR/wall_follow.log"

        exit 1

    fi


    if ! kill -0 "$MAP_DONE_WATCH_PID" 2>/dev/null
    then

        if grep -q "data: true" "$MAP_DONE_LOG"
        then

            echo
            echo "=========================================="
            echo "STAGE 1 COMPLETE"
            echo "=========================================="

            break

        else

            echo
            echo "ERROR:"
            echo "/map_run_finished watcher stopped unexpectedly."

            cat "$MAP_DONE_LOG"

            exit 1

        fi

    fi


    sleep 1

done


# ================================================================
# Verify Newly Saved Files
# ================================================================

if [ ! -s "$MAP_FILE" ]
then

    echo
    echo "ERROR:"
    echo "factory_map.pbstream was not generated."

    exit 1

fi


if [ ! -s "$PATH_FILE" ]
then

    echo
    echo "ERROR:"
    echo "patrol_path.csv was not generated."

    exit 1

fi


echo
echo "NEW MAP:"
echo "$MAP_FILE"

echo
echo "NEW PATH:"
echo "$PATH_FILE"


# ================================================================
# Stop Stage 1 Drive Node
# ================================================================

echo
echo "Stopping Wall Follow Node..."


kill_process_group "$DRIVE_PID"

DRIVE_PID=""


sleep 2


# ================================================================
# Stop Mapping Cartographer + Grid
# ================================================================

echo
echo "Stopping Mapping Cartographer..."


kill_process_group "$GRID_PID"

GRID_PID=""


kill_process_group "$CARTO_PID"

CARTO_PID=""


sleep 3


# ================================================================
# STAGE 1 -> STAGE 2 Transition
# ================================================================

echo
echo
echo "################################################"
echo "#                                              #"
echo "#       STAGE 1 -> STAGE 2 TRANSITION         #"
echo "#                                              #"
echo "################################################"
echo
echo "Map and path saved successfully."
echo
echo "Starting Pure Localization in 5 seconds..."


sleep 5


# ================================================================
# [8] Cartographer Pure Localization
# ================================================================

echo
echo "=========================================="
echo "[8] STARTING PURE LOCALIZATION"
echo "=========================================="


setsid ros2 run \
    cartographer_ros \
    cartographer_node \
    -configuration_directory "$BASE_DIR" \
    -configuration_basename cartographer_localization.lua \
    -load_state_filename "$MAP_FILE" \
    -load_frozen_state true \
    > "$LOG_DIR/cartographer_localization.log" 2>&1 &


CARTO_PID=$!


sleep 8


if ! kill -0 "$CARTO_PID" 2>/dev/null
then

    echo
    echo "ERROR: Cartographer Localization terminated."

    cat "$LOG_DIR/cartographer_localization.log"

    exit 1

fi


echo "Cartographer Localization running."


# ================================================================
# [9] Localization Occupancy Grid
# ================================================================

setsid ros2 run \
    cartographer_ros \
    cartographer_occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 \
    > "$LOG_DIR/grid_localization.log" 2>&1 &


GRID_PID=$!


sleep 5


if ! kill -0 "$GRID_PID" 2>/dev/null
then

    echo
    echo "ERROR: Localization Occupancy Grid terminated."

    exit 1

fi


# ================================================================
# [10] Wait for map -> base_link
# ================================================================

echo
echo "=========================================="
echo "[10] WAITING FOR LOCALIZATION"
echo "=========================================="


TF_READY=0


for i in $(seq 1 30)
do

    TF_RESULT=$(

        timeout 3 \
        ros2 run \
        tf2_ros \
        tf2_echo \
        map \
        base_link \
        2>/dev/null || true

    )


    if echo "$TF_RESULT" \
        | grep -q "Translation:"
    then

        TF_READY=1

        echo "map -> base_link READY"

        break

    fi


    echo "Waiting for map -> base_link..."

    sleep 1

done


if [ "$TF_READY" -ne 1 ]
then

    echo
    echo "ERROR:"
    echo "Localization could not be established."
    echo
    echo "STAGE 2 WILL NOT START."

    exit 1

fi


# ================================================================
# STAGE 2
# ================================================================

echo
echo
echo "################################################"
echo "#                                              #"
echo "#           STAGE 2 : PATROL-RUN              #"
echo "#                                              #"
echo "################################################"
echo
echo "Pure Pursuit"
echo "Latest 1st-Run Path"
echo "YOLO Safety Detection"
echo


# ================================================================
# Start Pure Pursuit
#
# STAGE 2 완료 조건:
# patrol_path.csv의 마지막 waypoint까지
# 0.10 m (10 cm) 이내로 접근
# ================================================================

setsid python3 \
    "$PATH_NODE" \
    --ros-args \
    -p path_file:="$PATH_FILE" \
    -p start_enabled:=true \
    -p loop_path:=false \
    -p max_start_distance:=1.10 \
    -p goal_tolerance:=0.10 \
    > "$LOG_DIR/path_follow.log" 2>&1 &


DRIVE_PID=$!


sleep 3


if ! kill -0 "$DRIVE_PID" 2>/dev/null
then

    echo
    echo "ERROR:"
    echo "Path Follow Node terminated."

    cat "$LOG_DIR/path_follow.log"

    exit 1

fi


echo
echo "=========================================="
echo "STAGE 2 STARTED"
echo "=========================================="
echo
echo "The robot is now following the path"
echo "recorded during THIS Stage 1 run."
echo
echo "YOLO box/cable safety is active."
echo


# ================================================================
# Main Monitor
# ================================================================

while true
do

    if ! kill -0 "$DRIVE_PID" 2>/dev/null
    then

        echo
        echo "=========================================="
        echo "STAGE 2 PATH FOLLOW NODE STOPPED"
        echo "=========================================="

        break

    fi


    if ! kill -0 "$LIDAR_PID" 2>/dev/null
    then

        echo
        echo "ERROR: RPLIDAR stopped."

        exit 1

    fi


    if ! kill -0 "$BRIDGE_PID" 2>/dev/null
    then

        echo
        echo "ERROR: Serial Bridge stopped."

        exit 1

    fi


    if ! kill -0 "$YOLO_PID" 2>/dev/null
    then

        echo
        echo "ERROR: YOLO Safety stopped."

        exit 1

    fi


    if ! kill -0 "$CARTO_PID" 2>/dev/null
    then

        echo
        echo "ERROR: Cartographer Localization stopped."

        exit 1

    fi


    sleep 1

done


echo
echo "=========================================="
echo "UGV02 2-PASS RUN COMPLETE"
echo "=========================================="

exit 0