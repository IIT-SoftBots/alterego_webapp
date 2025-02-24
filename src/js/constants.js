// ROS related commands
export const ROS_COMMANDS = {
    SETUP: 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash',
    CLEANUP: 'source /opt/ros/noetic/setup.bash && rosnode kill -a && killall -9 rosmaster',
    CLEAR_LOG: 'truncate -s 0 ~/catkin_ws/src/AlterEGO_v2/alterego_robot/config/SystemCheck.txt'
};

// Launch commands for different ROS nodes
export const LAUNCH_COMMANDS = {
    ROSCORE: 'roscore',
    USB_DETECTOR: 'rosrun alterego_robot usb_ports_detector.py',
    IMU: 'roslaunch alterego_robot imu.launch AlterEgoVersion:=2',
    PILOT: 'roslaunch alterego_robot pilot.launch AlterEgoVersion:=2',
    BACKWARD: 'roslaunch alterego_backward_controller backward.launch AlterEgoVersion:=2',
    BODY_ACTIVATION: 'roslaunch alterego_robot body_activation.launch AlterEgoVersion:=2',
    BODY_MOVEMENT: 'roslaunch alterego_robot body_movement.launch AlterEgoVersion:=2',
    WHEELS: 'roslaunch alterego_robot wheels.launch AlterEgoVersion:=2'
};

// UI States
export const UI_STATES = {
    POWER_OFF: {
        color: '#ef4444',
        text: 'Power: Off'
    },
    POWER_ON: {
        color: '#22c55e',
        text: 'Power: On'
    },
    SYSTEM_READY: {
        color: '#eab308',
        text: 'Status: Ready'
    },
    SYSTEM_RUNNING: {
        color: '#22c55e',
        text: 'Status: Running'
    }
};