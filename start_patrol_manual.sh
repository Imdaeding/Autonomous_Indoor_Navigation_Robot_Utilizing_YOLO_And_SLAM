#!/bin/bash

# set -u

# ================================================================
# ROS Environment
# ================================================================

source /opt/ros/humble/setup.bash
source /home/mice/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/mice/cyclonedds.xml
export ROS_DOMAIN_ID=0


BASE_DIR="/home/mice/ugv02_slam_test"

MAP_FILE="$BASE_DIR/maps/factory_map.pbstream"

PATH_FILE="$BASE_DIR/maps/patrol_path.csv"

LIDAR_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

ESP_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C37186196-if00"

YOLO_PYTHON="/home/mice/yolo_practice/.venv/bin/python"

LOG_DIR="$BASE_DIR/patrol_logs"

mkdir -p "$LOG_DIR"


# ================================================================
# PID
# ================================================================

LIDAR_PID=""

BRIDGE_PID=""

YOLO_PID=""

TF_PID=""

CARTO_PID=""

GRID_PID=""

PATH_PID=""


# ================================================================
# Cleanup
# ================================================================

cleanup()
{
    echo
    echo "=========================================="
    echo "PATROL SYSTEM SHUTDOWN"
    echo "=========================================="

    if [ -n "$PATH_PID" ]; then

        kill "$PATH_PID" \
            2>/dev/null || true

    fi

    sleep 0.5


    if [ -n "$YOLO_PID" ]; then

        kill "$YOLO_PID" \
            2>/dev/null || true

    fi


    if [ -n "$GRID_PID" ]; then

        kill "$GRID_PID" \
            2>/dev/null || true

    fi


    if [ -n "$CARTO_PID" ]; then

        kill "$CARTO_PID" \
            2>/dev/null || true

    fi


    if [ -n "$TF_PID" ]; then

        kill "$TF_PID" \
            2>/dev/null || true

    fi


    if [ -n "$BRIDGE_PID" ]; then

        kill "$BRIDGE_PID" \
            2>/dev/null || true

    fi


    if [ -n "$LIDAR_PID" ]; then

        kill "$LIDAR_PID" \
            2>/dev/null || true

    fi

    sleep 1

    echo "Patrol nodes stopped."
}


trap cleanup EXIT INT TERM


# ================================================================
# Existing Map-Run Service
# ================================================================

if systemctl is-active --quiet \
    ugv02-autonomous.service
then

    echo
    echo "ERROR:"
    echo "ugv02-autonomous.service is running."
    echo
    echo "Stop it first:"
    echo
    echo "sudo systemctl stop ugv02-autonomous.service"

    exit 1

fi


# ================================================================
# Required Files
# ================================================================

if [ ! -f "$MAP_FILE" ]
then

    echo "ERROR:"
    echo "factory_map.pbstream not found."
    echo "$MAP_FILE"

    exit 1

fi


if [ ! -f "$PATH_FILE" ]
then

    echo "ERROR:"
    echo "patrol_path.csv not found."
    echo "$PATH_FILE"

    exit 1

fi


if [ ! -f "$BASE_DIR/cartographer_localization.lua" ]
then

    echo "ERROR:"
    echo "cartographer_localization.lua not found."

    exit 1

fi


if [ ! -f "$BASE_DIR/path_follow_node.py" ]
then

    echo "ERROR:"
    echo "path_follow_node.py not found."

    exit 1

fi


echo
echo "=========================================="
echo "UGV02 PATROL SYSTEM"
echo "=========================================="

echo
echo "Map:"
echo "$MAP_FILE"

echo
echo "Path:"
echo "$PATH_FILE"


# ================================================================
# RPLIDAR
# ================================================================

echo
echo "[1] Waiting for RPLIDAR..."

while [ ! -e "$LIDAR_PORT" ]
do

    echo "Waiting for RPLIDAR..."

    sleep 1

done

echo "RPLIDAR detected:"
echo "$LIDAR_PORT"


# ================================================================
# ESP32
# ================================================================

echo
echo "[2] Waiting for ESP32..."

while [ ! -e "$ESP_PORT" ]
do

    echo "Waiting for ESP32..."

    sleep 1

done

echo "ESP32 detected:"
echo "$ESP_PORT"


# ================================================================
# Static TF
# ================================================================

echo
echo "[3] Starting Static TF..."

ros2 run \
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


# ================================================================
# RPLIDAR Node
# ================================================================

echo
echo "[4] Starting RPLIDAR..."

ros2 launch \
    sllidar_ros2 \
    sllidar_a1_launch.py \
    serial_port:="$LIDAR_PORT" \
    serial_baudrate:=115200 \
    frame_id:=laser_link \
    > "$LOG_DIR/lidar.log" 2>&1 &

LIDAR_PID=$!

sleep 3


if ! kill -0 "$LIDAR_PID" \
    2>/dev/null
then

    echo "ERROR: RPLIDAR process terminated."

    echo "Check:"
    echo "cat $LOG_DIR/lidar.log"

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

    echo "ERROR: /scan not received."

    echo "Check:"
    echo "cat $LOG_DIR/lidar.log"

    exit 1

fi


# ================================================================
# Serial Bridge
# ================================================================

echo
echo "[5] Starting Serial Bridge..."

python3 \
    "$BASE_DIR/ugv02_serial_bridge.py" \
    --ros-args \
    -p port:="$ESP_PORT" \
    -p baudrate:=115200 \
    > "$LOG_DIR/bridge.log" 2>&1 &

BRIDGE_PID=$!

sleep 3


if ! kill -0 "$BRIDGE_PID" \
    2>/dev/null
then

    echo "ERROR: Serial Bridge terminated."

    echo "Check:"
    echo "cat $LOG_DIR/bridge.log"

    exit 1

fi


echo "Serial Bridge READY"


# ================================================================
# YOLO
# ================================================================

echo
echo "[6] Starting YOLO Safety..."

"$YOLO_PYTHON" \
    "$BASE_DIR/yolo_safety_node.py" \
    > "$LOG_DIR/yolo.log" 2>&1 &

YOLO_PID=$!


echo "Waiting for Vision Safety..."

VISION_READY=0


for i in $(seq 1 60)
do

    if ! kill -0 "$YOLO_PID" \
        2>/dev/null
    then

        echo "ERROR: YOLO node terminated."

        echo "Check:"
        echo "cat $LOG_DIR/yolo.log"

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

    echo "ERROR:"
    echo "Vision Safety did not become READY."

    echo "Check:"
    echo "cat $LOG_DIR/yolo.log"

    exit 1

fi


echo "YOLO Safety READY"


# ================================================================
# Cartographer Localization
# ================================================================

echo
echo "[7] Starting Cartographer Localization..."

ros2 run \
    cartographer_ros \
    cartographer_node \
    -configuration_directory "$BASE_DIR" \
    -configuration_basename cartographer_localization.lua \
    -load_state_filename "$MAP_FILE" \
    -load_frozen_state true \
    > "$LOG_DIR/cartographer.log" 2>&1 &

CARTO_PID=$!


sleep 8


if ! kill -0 "$CARTO_PID" \
    2>/dev/null
then

    echo "ERROR:"
    echo "Cartographer Localization terminated."

    echo "Check:"
    echo "cat $LOG_DIR/cartographer.log"

    exit 1

fi


echo "Cartographer Localization running."


# ================================================================
# Occupancy Grid
# ================================================================

echo
echo "[8] Starting Occupancy Grid..."

ros2 run \
    cartographer_ros \
    cartographer_occupancy_grid_node \
    -resolution 0.05 \
    > "$LOG_DIR/grid.log" 2>&1 &

GRID_PID=$!


sleep 5


if ! kill -0 "$GRID_PID" \
    2>/dev/null
then

    echo "ERROR:"
    echo "Occupancy Grid terminated."

    exit 1

fi


echo "Occupancy Grid running."


# ================================================================
# Wait for map -> base_link Localization TF
# ================================================================

echo
echo "[9] Waiting for Localization TF..."

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
    echo "WARNING:"
    echo "Localization TF was not confirmed."
    echo "Robot will remain DISARMED."
    echo

fi

# ================================================================
# Path Follow
#
# start_enabled is deliberately FALSE.
# ================================================================

echo
echo "[10] Starting Path Follow Node..."

python3 \
    "$BASE_DIR/path_follow_node.py" \
    --ros-args \
    -p path_file:="$PATH_FILE" \
    -p start_enabled:=false \
    > "$LOG_DIR/path_follow.log" 2>&1 &

PATH_PID=$!


sleep 3


if ! kill -0 "$PATH_PID" \
    2>/dev/null
then

    echo "ERROR:"
    echo "Path Follow Node terminated."

    echo "Check:"
    echo "cat $LOG_DIR/path_follow.log"

    exit 1

fi


echo
echo "=========================================="
echo "PATROL SYSTEM READY"
echo "=========================================="

echo
echo "ROBOT IS CURRENTLY DISARMED."
echo
echo "DO NOT START UNTIL LOCALIZATION IS CHECKED."

echo
echo "RViz:"
echo "  Fixed Frame = map"
echo "  Map       = /map"
echo "  LaserScan = /scan"
echo "  Path      = /patrol_path"
echo "  TF"

echo
echo "To START:"
echo
echo "ros2 param set /path_follow_node start_enabled true"

echo
echo "To STOP:"
echo
echo "ros2 param set /path_follow_node start_enabled false"

echo
echo "=========================================="


# ================================================================
# Monitor
# ================================================================

while true
do

    for PID in \
        "$LIDAR_PID" \
        "$BRIDGE_PID" \
        "$YOLO_PID" \
        "$TF_PID" \
        "$CARTO_PID" \
        "$GRID_PID" \
        "$PATH_PID"

    do

        if ! kill -0 "$PID" \
            2>/dev/null
        then

            echo
            echo "ERROR:"
            echo "A Patrol process terminated."
            echo
            echo "Stopping Patrol system."

            exit 1

        fi

    done

    sleep 1

done