#!/bin/bash
# start_vio_sim.sh - Opens terminals for the VIO SITL stack
# (iris_with_vio_camera in iris_warehouse_vio.sdf)
#
# Usage: bash scripts/start_vio_sim.sh [--start | --stop | --help]
#   --start   open the Gazebo, ArduPilot, MAVROS, Bridge and IMU Bridge tabs
#   --stop    stop the sim (also stops start_all.sh) and close its tabs
#   no option shows this help, so running the script by itself never
#   launches or builds anything
#
# The same script lives in vin_autonomy/scripts and udl_aakash/scripts.
# ArduPilot runs from vin_autonomy/runs/sitl so its eeprom.bin and logs stay
# separate from udl_aakash's start_all.sh and gnc_autonomy's runs.
# The ArduPilot, Gazebo and MAVROS installs are udl_aakash's, used as-is.

PROJECT_DIR="/data/projects/udl_aakash"
ARDUPILOT_DIR="/data/projects/udl_aakash/tools/ardupilot/Tools/autotest"
VIN_DIR="/data/projects/vin_autonomy"
RUN_DIR="$VIN_DIR/runs/sitl"
IMU_BRIDGE="$VIN_DIR/vin_ws/src/gz_imu_bridge/gz_imu_bridge.py"

show_help() {
    echo "Usage: bash $(basename "$0") [--start | --stop | --help]"
    echo
    echo "  --start   open the VIO sim in terminal tabs:"
    echo "            Gazebo (iris_warehouse_vio.sdf), ArduPilot SITL (--no-rebuild),"
    echo "            MAVROS, ros_gz bridge (camera, ground truth, clock) and IMU bridge"
    echo "  --stop    stop the sim and close its tabs: Gazebo, ArduPilot, MAVProxy,"
    echo "            MAVROS, bridges and rqt_image_view (also stops start_all.sh)"
    echo "  --help    show this help"
    echo
    echo "With no option this help is shown and nothing is started or built."
}

start_sim() {
    mkdir -p "$RUN_DIR"

    # Tab 1: Gazebo
    gnome-terminal --tab --title="Gazebo" -- bash -c "cd $PROJECT_DIR && gz sim -v4 -r iris_warehouse_vio.sdf; bash"

    # Small delay so the world is loaded before SITL connects to the plugin
    sleep 5

    # Tab 2: ArduPilot (--no-rebuild: use the existing build, never compile)
    gnome-terminal --tab --title="ArduPilot" -- bash -c "cd $RUN_DIR && $ARDUPILOT_DIR/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild --map --console -L Basingstoke; bash"

    # Small delay before starting dependent services
    sleep 3

    # Tab 3: MAVROS
    gnome-terminal --tab --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args -p system_id:=200 -p fcu_url:=udp://:14550@localhost:14555; bash"

    # Tab 4: Bridge (camera, ground truth, sim clock)
    # The VIO IMU is not bridged here: this ros_gz_bridge is built for Gazebo
    # Garden and corrupts Harmonic IMU messages. Tab 5 handles it instead.
    gnome-terminal --tab --title="Bridge" -- bash -c "ros2 run ros_gz_bridge parameter_bridge \
/vio/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
/vio/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/vio/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock; bash"

    # Tab 5: IMU bridge (Harmonic IMU -> ROS 2, see gz_imu_bridge.py)
    gnome-terminal --tab --title="IMU Bridge" -- bash -c "python3 $IMU_BRIDGE; bash"
}

# Stop logic below is the same as udl_aakash/scripts/start_all.sh --stop

# Programs the sim tabs run (matched against the full command line)
SIM_PATTERN='^gz sim|sim_vehicle\.py|mavproxy\.py|bin/arducopter|mavros_node|parameter_bridge|rqt_image_view|gz_imu_bridge\.py'
# Same programs as they appear inside a tab's "bash -c" command
TAB_PATTERN='gz sim|sim_vehicle\.py|mavros_node|parameter_bridge|rqt_image_view|gz_imu_bridge\.py'

# PIDs matching the pattern whose program really is a sim program. A process
# that only mentions these names (a grep, an editor, another shell) is skipped.
find_programs() {
    local argv0 argv1
    for pid in $(pgrep -f "$SIM_PATTERN"); do
        [ "$pid" = "$$" ] && continue
        { IFS= read -r -d '' argv0; IFS= read -r -d '' argv1; } < /proc/$pid/cmdline 2>/dev/null || continue
        argv0="${argv0%% *}"  # gz renames itself "gz sim server", keep the first word
        case "${argv0##*/}" in
            gz|arducopter|parameter_bridge|mavros_node) echo "$pid" ;;
            python|python3)
                case "${argv1##*/}" in
                    sim_vehicle.py|mavproxy.py|ros2|gz_imu_bridge.py|rqt_image_view) echo "$pid" ;;
                esac ;;
        esac
    done
}

# The "bash -c '<program>; bash'" shells that hold each terminal tab open
find_tab_shells() {
    for pid in $(pgrep -f "^bash -c .*($TAB_PATTERN)"); do
        [ "$pid" = "$$" ] || echo "$pid"
    done
}

# Wait up to $1 seconds for the given PIDs to exit
wait_for_exit() {
    local secs=$1; shift
    for _ in $(seq "$secs"); do
        kill -0 "$@" 2>/dev/null || return 0
        sleep 1
    done
    return 1
}

stop_sim() {
    # Note the tab shells first: once their program exits they turn into a
    # plain "bash" and can no longer be told apart from other terminals
    local tabs progs
    tabs=$(find_tab_shells)
    progs=$(find_programs)

    if [ -z "$progs" ] && [ -z "$tabs" ]; then
        echo "Nothing running."
        return 0
    fi

    if [ -n "$progs" ]; then
        echo "Stopping:"
        for pid in $progs; do
            echo "  $pid $(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | cut -c1-80)"
        done

        # Ctrl-C first so everything shuts down cleanly
        kill -INT $progs 2>/dev/null
        if ! wait_for_exit 15 $progs; then
            progs=$(find_programs)
            echo "Still running after Ctrl-C, sending TERM:" $progs
            kill -TERM $progs 2>/dev/null
            if ! wait_for_exit 10 $progs; then
                progs=$(find_programs)
                echo "Still running after TERM, sending KILL:" $progs
                kill -KILL $progs 2>/dev/null
            fi
        fi
    fi

    # Close the terminal tabs
    if [ -n "$tabs" ]; then
        kill -HUP $tabs 2>/dev/null
        wait_for_exit 5 $tabs
    fi

    progs=$(find_programs)
    if [ -n "$progs" ]; then
        echo "Could not stop:" $progs
        return 1
    fi
    echo "Sim stopped."
}

case "$1" in
    --start) start_sim ;;
    --stop)  stop_sim ;;
    --help|-h|"") show_help ;;
    *) echo "Unknown option: $1"; echo; show_help; exit 1 ;;
esac
