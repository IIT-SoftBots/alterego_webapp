// ROS related commands
export const NUC_BASE_IP    = '192.168.0.110';      // Check and modify same const in webapp.js
export const ROS_CATKIN_WS  = '~/catkin_ws';  // Default: '~/catkin_ws'
export const ROS_SRC_FOLDER = '/src/AlterEGO_Adriano';  // Default: '/src'
export const ROS_CATKIN_WS_LOCAL = '~/AlterEGO_Adriano/catkin_ws';  // Default: '~/catkin_ws'
export const ROS_MASTER_URI = 'export ROS_MASTER_URI=http://192.168.0.110:11311';  // Default: 'export ROS_MASTER_URI=http://localhost:11311'
export const ROS_IP         = 'export ROS_IP=192.168.0.110'
export const ROS_HOSTNAME   = 'export ROS_HOSTNAME=192.168.0.110'
export const ROS_IP_LOCAL         = 'export ROS_IP=192.168.0.111'
export const ROS_HOSTNAME_LOCAL   = 'export ROS_HOSTNAME=192.168.0.111'

export const ROS_COMMANDS   = {
    SETUP: ROS_MASTER_URI + ' && ' + ROS_IP + ' && ' + ROS_HOSTNAME + ' && source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS + '/devel/setup.bash',
    SETUP_LOCAL: ROS_MASTER_URI + ' && ' + ROS_IP_LOCAL + ' && ' + ROS_HOSTNAME_LOCAL + ' &&  source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS_LOCAL + '/devel/setup.bash',
    CLEANUP: 'source /opt/ros/noetic/setup.bash && rosnode kill -a && killall -9 rosmaster',
    CLEAR_LOG: 'truncate -s 0 ' + ROS_CATKIN_WS + ROS_SRC_FOLDER + '/alterego_robot/config/SystemCheck.txt'
};

export const SOUND_PATH_LOCAL = '~/AlterEGO_Adriano/EGO_GUI/config/low_battery.mp3'

// Workflow Routines
export const STATE = {
    RESTART_AUTO:              -1,
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

    // Core Nodes
    ROSCORE: 'roscore',
    USB_DETECTOR: 'rosrun alterego_robot usb_ports_detector.py',
    IMU: 'roslaunch alterego_robot imu.launch AlterEgoVersion:=2',
    BATTERY: 'roslaunch alterego_robot battery_status.launch AlterEgoVersion:=2',
    BATTERY_STANDALONE: 'roslaunch alterego_robot battery_status.launch AlterEgoVersion:=2 standalone:=true',
    DOCKING: 'roslaunch alterego_docking_controller docking.launch AlterEgoVersion:=2',
    WHEELS: 'roslaunch alterego_robot wheels.launch AlterEgoVersion:=2',
    
    // Body Movement Nodes
    PILOT: 'roslaunch alterego_robot pilot.launch AlterEgoVersion:=2',    
    BODY_ACTIVATION: 'roslaunch alterego_robot body_activation.launch AlterEgoVersion:=2',
    BODY_MOVEMENT: 'roslaunch alterego_robot body_movement.launch AlterEgoVersion:=2',
    
    // Additional Nodes
    FACE_EXPRESSION: 'roslaunch alterego_robot face_expressions.launch AlterEgoVersion:=2',
    FACE_RECOGNITION: 'roslaunch alterego_face_recognition face_recognition.launch',
    FACE_TRACKING: 'roslaunch alterego_face_tracking face_tracking.launch',
    STT: 'roslaunch alterego_conversation speech2text.launch 2>/dev/null',
    TTS: 'roslaunch alterego_text2speech text2speech.launch',
    KILL_SPEECH: './home/alterego-vision/kill_speech.sh',
    NAVIGATION: 'roslaunch alterego_navigation autonomous_nav.launch',
    SAY_TIRED: 'roslaunch alterego_adjust_docking say_tired.launch',

    // Play ROSBAGS
    BREATH:'roslaunch alterego_rosbags_play play_breath.launch',
    
    // Additional constants
    TARGET_LOC: 'Mostra1',
    DOCK_STATION: 'DockStation',    
    ADJUST_DOCKING: '/adjust_docking',
    STOP_BREATH: '/rosbag_play',

    // Stop Core Nodes
    STOP_BATTERY_STANDALONE: {
        BATTERY:        '/battery/battery_status',
        QB_INTERFACE:   '/wheels/qb_interface_node'
    },

    // Stop Body Movement Nodes
    STOP_PILOT: {
        R_CTRL: '/right/arms_compliant_control_node',
        L_CTRL: '/left/arms_compliant_control_node',
        INBOUND:'/inbound_data',
        SOCKET: '/socket'
    },
    STOP_BODY_ACTIVATION: {
        R_ARM:  '/right/qb_manager',
        L_ARM:  '/left/qb_manager',        
    },
    STOP_BODY_MOVEMENT: {
        R_ARM:  '/right/arm_inv_dyn',
        L_ARM:  '/left/arm_inv_dyn',
        HEAD:   '/head/head_inv_kin',
        R_MAIN: '/right/arm_inv_kin_main',
        L_MAIN: '/left/arm_inv_kin_main',
        PITCH:  '/pitch_correction'
    },

    // Stop Additional Nodes
    STOP_TTS:               '/text2speech',
    STOP_STT:               '/speech2text',
    STOP_FACE_EXPRESSION:   '/face_expressions',
    STOP_FACE_RECOGNITION:  '/face_recognition',
    STOP_FACE_TRACKING:     '/face_tracker',
    
    // Stop Naviagation Nodes
    STOP_NAVIGATION: {
        AMCL: '/amcl',
        LIDAR: '/lidar',
        MOVE_BASE: '/move_base',
        MAP_SERVER: '/map_server',
        MAP_SERVER_OBSTACLE: '/map_server_obstacle',
        NAVIGATION: '/navigation',
        VIS_ROBOT: '/visualize_robot'
    }

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