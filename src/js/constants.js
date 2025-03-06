// ROS related commands
export const NUC_BASE_IP    = '192.168.178.80';         // Check and modify same const in webapp.js
export const ROS_CATKIN_WS  = '~/AlterEgo_Adriano_ws';  // Default: '~/catkin_ws'
export const ROS_SRC_FOLDER = '/src/AlterEGO_Adriano';  // Default: '/src'
export const ROS_COMMANDS   = {
    SETUP: 'source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS + '/devel/setup.bash',
    SETUP_LOCAL: 'source ~/.bashrc && source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS + '/devel/setup.bash',
    CLEANUP: 'source /opt/ros/noetic/setup.bash && rosnode kill -a && killall -9 rosmaster',
    CLEAR_LOG: 'truncate -s 0 ' + ROS_CATKIN_WS + ROS_SRC_FOLDER + '/alterego_robot/config/SystemCheck.txt'
};

// Workflow Routines
export const STATE = {
    INIT:                       0,
    ACTIVATE_ROBOT:             1,
    STAND_UP:                   2,
    WORK_MODE:                  3,
    PAUSED:                     4,
    DOCKED:                     5,
    RECOVERY_FROM_EMERGENCY:    6,
    POWER_OFF_NUCS:             7
};

// Launch commands for different ROS nodes
export const LAUNCH_COMMANDS = {
    ROSCORE: 'roscore',
    USB_DETECTOR: 'rosrun alterego_robot usb_ports_detector.py',
    IMU: 'roslaunch alterego_robot imu.launch AlterEgoVersion:=2',
    BATTERY: 'roslaunch alterego_robot battery_status.launch AlterEgoVersion:=2',
    PILOT: 'roslaunch alterego_robot pilot.launch AlterEgoVersion:=2',
    BACKWARD: 'roslaunch alterego_docking_controller docking.launch AlterEgoVersion:=2 movementDirection:="backward" maxLinDistance:=0.3',
    FORWARD: 'roslaunch alterego_docking_controller docking.launch AlterEgoVersion:=2 movementDirection:="forward"',
    BODY_ACTIVATION: 'roslaunch alterego_robot body_activation.launch AlterEgoVersion:=2',
    BODY_MOVEMENT: 'roslaunch alterego_robot body_movement.launch AlterEgoVersion:=2',
    WHEELS: 'roslaunch alterego_robot wheels.launch AlterEgoVersion:=2',
    FACE_EXPRESSION: 'roslaunch alterego_robot face_expressions.launch  AlterEgoVersion:=2',
    FACE_RECOGNITION: 'roslaunch alterego_face_recognition face_recognition.launch',
    FACE_TRACKING: 'roslaunch alterego_face_tracking face_tracking.launch'
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

// UI Strings, add localization, further use
// export const TEXT_EN = {
//     START_ROBOT_STR:        'Start Robot',
//     PAUSE_STR:              'Pause',
//     PLAY_STR:               'Play',
//     HOME_STR:               'Home',
//     POWER_OFF_STR:          'Power Off',
//     SETTINGS_STR:           'Settings',
//     CLOSE_STR:              'Close App',
//     ADMIN_MENU_STR:         'Admin Menu',
//     SYSTEM_STATUS_STR:      'System Status',
//     SS_POWER_STR:           'Power: Off',
//     SS_BATTERY_STR:         'Battery',
//     SS_STATUS_STR:          'Status: Ready'
// }