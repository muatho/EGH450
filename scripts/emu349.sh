# EGH450 emulator analogue of run349.
# Deliberately excludes control.launch, environment.launch, MAVROS/Pixhawk,
# Vicon, and the physical payload controller.

function egh_emulator_mission_preflight() {
    local failed=0
    local services topics message arm_response

    echo
    echo "========================================"
    echo "EGH450 EMULATOR MISSION READINESS CHECK"
    echo "========================================"

    if rosparam get /run_id >/dev/null 2>&1; then
        echo "[PASS] ROS master"
        services="$(rosservice list 2>/dev/null)"
        topics="$(rostopic list 2>/dev/null)"
    else
        echo "[FAIL] ROS master"
        failed=1
    fi

    if grep -Fxq "/breadcrumb/request_path" <<< "$services"; then
        echo "[PASS] Breadcrumb service"
    else
        echo "[FAIL] Breadcrumb service"
        failed=1
    fi

    # A completed emulated landing automatically disarms the vehicle. Re-arm
    # before every emulator mission so the mission command can be rerun without
    # rebuilding the entire tmux session. This service exists only in the
    # emulator and does not alter the real-aircraft run349 workflow.
    if grep -Fxq "/uavasr/cmd/arming" <<< "$services" &&
       arm_response="$(rosservice call /uavasr/cmd/arming "value: true" 2>/dev/null)" &&
       grep -Eq '^success: (True|true)$' <<< "$arm_response"; then
        echo "[PASS] Emulator arm request"
    else
        echo "[FAIL] Emulator could not be armed"
        failed=1
    fi

    message=""
    if grep -Fxq "/uavasr/state" <<< "$topics"; then
        message="$(timeout 5 rostopic echo -n 1 /uavasr/state 2>/dev/null)"
    fi

    if grep -Eq '^connected: (True|true)$' <<< "$message" &&
       grep -Eq '^armed: (True|true)$' <<< "$message" &&
       grep -Eq '^mode: "?OFFBOARD"?$' <<< "$message"; then
        echo "[PASS] Emulator connected, armed and OFFBOARD"
    else
        echo "[FAIL] Emulator is not connected, armed and OFFBOARD"
        failed=1
    fi

    if grep -Fxq "/spar/flight/status" <<< "$topics"; then
        echo "[PASS] SPAR action server"
    else
        echo "[FAIL] SPAR action server"
        failed=1
    fi

    if grep -Fxq "/uavasr/odom" <<< "$topics" &&
       message="$(timeout 5 rostopic echo -n 1 /uavasr/odom 2>/dev/null)" &&
       [[ -n "$message" ]]; then
        echo "[PASS] Emulator pose"
    else
        echo "[FAIL] Emulator pose"
        failed=1
    fi

    if grep -Fxq "/uavasr/battery" <<< "$topics" &&
       message="$(timeout 5 rostopic echo -n 1 /uavasr/battery 2>/dev/null)" &&
       [[ -n "$message" ]]; then
        echo "[PASS] Emulator battery"
    else
        echo "[WARN] Emulator battery unavailable"
    fi

    echo "========================================"
    if (( failed )); then
        echo "MISSION NOT STARTED."
        echo "Inspect every failed item above."
        echo "For a clean restart, kill cdrone and run emu349 again."
        return 1
    fi
    echo "ALL EMULATOR MISSION PREREQUISITES PASSED."
}

function emu349() {
    disros
    tmux kill-session -t cdrone 2>/dev/null
    echo "Starting tmux session for EGH450 UAV emulation"

    tmux new-session -d -s cdrone
    tmux set-option -t cdrone -g mouse on
    tmux set-option -t cdrone remain-on-exit on
    tmux set-option -w -t cdrone:0 window-size largest

    local col1 col2 col3
    col1="$(tmux display-message -p -t cdrone:0.0 '#{pane_id}')"
    col2="$(tmux split-window -t "$col1" -h -P -F '#{pane_id}')"
    col3="$(tmux split-window -t "$col2" -h -P -F '#{pane_id}')"

    local c1_status c1_breadcrumb c1_htop
    c1_status="$(tmux split-window -t "$col1" -v -p 85 -P -F '#{pane_id}')"
    c1_breadcrumb="$(tmux split-window -t "$c1_status" -v -p 71 -P -F '#{pane_id}')"
    c1_htop="$(tmux split-window -t "$c1_breadcrumb" -v -p 67 -P -F '#{pane_id}')"
    tmux send-keys -t "$col1" "roscore" C-m
    tmux send-keys -t "$c1_status" "egh_wait_for_ros_master 30 && python3 ~/catkin_ws_G6/scripts/egh_emulator_status_dashboard.py" C-m
    tmux send-keys -t "$c1_breadcrumb" "egh_wait_for_ros_master 30 && roslaunch ~/catkin_ws_G6/launch/breadcrumb.launch" C-m
    tmux send-keys -t "$c1_htop" "htop" C-m

    local c2_emulator c2_environment c2_vision
    c2_emulator="$(tmux split-window -t "$col2" -v -P -F '#{pane_id}')"
    c2_environment="$(tmux split-window -t "$c2_emulator" -v -P -F '#{pane_id}')"
    c2_vision="$(tmux split-window -t "$c2_environment" -v -P -F '#{pane_id}')"
    tmux send-keys -t "$col2" "echo 'Physical payload disabled in emulation.'" C-m
    tmux send-keys -t "$c2_emulator" "egh_wait_for_ros_master 30 && roslaunch spar_node spar_uavasr.launch deplete_battery:=false" C-m
    tmux send-keys -t "$c2_environment" "echo 'Map/environment are provided by spar_uavasr.launch.'" C-m
    tmux send-keys -t "$c2_vision" "egh_wait_for_ros_master 30 && roslaunch depthai_publisher image_topics.launch pose_source:=emulator" C-m

    local c3_mission c3_kill c3_pose c3_odom
    c3_mission="$(tmux split-window -t "$col3" -v -P -F '#{pane_id}')"
    c3_kill="$(tmux split-window -t "$col3" -v -P -F '#{pane_id}')"
    c3_pose="$(tmux split-window -t "$col3" -v -P -F '#{pane_id}')"
    c3_odom="$(tmux split-window -t "$col3" -v -P -F '#{pane_id}')"
    tmux send-keys -t "$col3" "egh_wait_for_ros_master 30 && recordbag" C-m
    tmux send-keys -t "$c3_mission" "egh_emulator_mission_preflight && roslaunch spar_node nav_mission.launch pose_source:=emulator deploy:=false launch_payload:=false enable_test_target_injection:=true boundary_checks_enabled:=true"
    tmux send-keys -t "$c3_kill" "echo 'Emergency stop: type tmux kill-session -t cdrone and press Enter only when required.'" C-m
    tmux send-keys -t "$c3_pose" "egh_wait_for_ros_master 30 && egh_wait_for_topic /uavasr/pose 30 && rostopic echo /uavasr/pose" C-m
    tmux send-keys -t "$c3_odom" "egh_wait_for_ros_master 30 && egh_wait_for_topic /uavasr/odom 30 && rostopic echo /uavasr/odom" C-m

    tmux select-pane -t "$c1_status"
    tmux attach-session -t cdrone
}

export -f egh_emulator_mission_preflight
export -f emu349
