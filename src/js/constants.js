// ROS related commands
// Use a placeholder value that will be updated at runtime
export let AlterEgoVersion = 2;
export let RobotName = 'robot_alterego';
export let NUC_BASE_IP = '192.168.88.110';  
export let NUC_VISION_IP = '192.168.88.111';  
export let ROS_MASTER_URI = `export ROS_MASTER_URI=http://${NUC_BASE_IP}:11311`;
export let ROS_IP = `export ROS_IP=${NUC_BASE_IP}`;
export let ROS_HOSTNAME = `export ROS_HOSTNAME=${NUC_BASE_IP}`;
export let ROS_IP_LOCAL = `export ROS_IP=${NUC_VISION_IP}`;
export let ROS_HOSTNAME_LOCAL = `export ROS_HOSTNAME=${NUC_VISION_IP}`;

export const ROS_CATKIN_WS = '~/catkin_ws';  // Default: '~/catkin_ws'
export const ROS_SRC_FOLDER = '/src/AlterEGO_v2';  // Default: '/src'
export const ROS_CATKIN_WS_LOCAL = '~/catkin_ws';  // Default: '~/catkin_ws'

// export const NUC_BASE_IP    = '192.168.0.110';      // Check and modify same const in webapp.js
// export const ROS_CATKIN_WS  = '~/catkin_ws';  // Default: '~/catkin_ws'
// export const ROS_SRC_FOLDER = '/src/AlterEGO_Adriano';  // Default: '/src'
// export const ROS_CATKIN_WS_LOCAL = '~/AlterEGO_Adriano/catkin_ws';  // Default: '~/catkin_ws'
// export const ROS_MASTER_URI = 'export ROS_MASTER_URI=http://192.168.0.110:11311';  // Default: 'export ROS_MASTER_URI=http://localhost:11311'
// export const ROS_IP         = 'export ROS_IP=192.168.0.110'
// export const ROS_HOSTNAME   = 'export ROS_HOSTNAME=192.168.0.110'
// export const ROS_IP_LOCAL         = 'export ROS_IP=192.168.0.111'
// export const ROS_HOSTNAME_LOCAL   = 'export ROS_HOSTNAME=192.168.0.111'

export let ROS_COMMANDS = {
    SETUP: ROS_MASTER_URI + ' && ' + ROS_IP + ' && ' + ROS_HOSTNAME + ' && source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS + '/devel/setup.bash',
    SETUP_LOCAL: ROS_MASTER_URI + ' && ' + ROS_IP_LOCAL + ' && ' + ROS_HOSTNAME_LOCAL + ' &&  source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS_LOCAL + '/devel/setup.bash',
    CLEANUP: 'source /opt/ros/noetic/setup.bash && rosnode kill -a && killall -9 rosmaster',
    CLEAR_LOG: 'truncate -s 0 ' + ROS_CATKIN_WS + ROS_SRC_FOLDER + '/alterego_robot/config/SystemCheck.txt'
};

// Function to update the IP address called in main.js
export async function initializeConfig() {
    try {
        const response = await fetch('/api/config');
        if (response.ok) {
            const config = await response.json();
            NUC_BASE_IP = config.NUC_BASE_IP;
            NUC_VISION_IP = config.NUC_VISION_IP || NUC_VISION_IP;
            const oldVersion = AlterEgoVersion;
            AlterEgoVersion = config.AlterEgoVersion || AlterEgoVersion;
            if (oldVersion !== AlterEgoVersion) {
                console.log(`AlterEgoVersion updated from ${oldVersion} to ${AlterEgoVersion}`);
            }
            RobotName = config.RobotName || RobotName;
            
            // Update derived values
            ROS_MASTER_URI = `export ROS_MASTER_URI=http://${NUC_BASE_IP}:11311`;
            ROS_IP = `export ROS_IP=${NUC_BASE_IP}`;
            ROS_HOSTNAME = `export ROS_HOSTNAME=${NUC_BASE_IP}`;
            ROS_IP_LOCAL = `export ROS_IP=${NUC_VISION_IP}`;
            ROS_HOSTNAME_LOCAL = `export ROS_HOSTNAME=${NUC_VISION_IP}`;
            
            // Update ROS_COMMANDS
            ROS_COMMANDS.SETUP = ROS_MASTER_URI + ' && ' + ROS_IP + ' && ' + ROS_HOSTNAME + ' && source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS + '/devel/setup.bash' + ' && export ROBOT_NAME=' + RobotName;
            ROS_COMMANDS.SETUP_LOCAL = ROS_MASTER_URI + ' && ' + ROS_IP_LOCAL + ' && ' + ROS_HOSTNAME_LOCAL + ' &&  source /opt/ros/noetic/setup.bash && source ' + ROS_CATKIN_WS_LOCAL + '/devel/setup.bash' + ' && export ROBOT_NAME=' + RobotName;
            
            // Aggiorna i comandi di lancio DOPO aver aggiornato AlterEgoVersion
            updateLaunchCommands();            
            console.log(`Config initialized with NUC_BASE_IP: ${NUC_BASE_IP}, NUC_VISION_IP: ${NUC_VISION_IP}`);
        }
    } catch (error) {
        console.error('Failed to initialize config:', error);
    }
}

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

// Launch commands for different ROS nodes - inizialmente vuoti
export let LAUNCH_COMMANDS = {
    // Core Nodes
    ROSCORE: 'roscore',
    USB_DETECTOR: 'rosrun alterego_robot usb_ports_detector.py',
    BATTERY_MONITOR: {
        START:      'cd ' + ROS_CATKIN_WS + ROS_SRC_FOLDER + '/utils/alterego_battery_status/ && ./battery_monitor.sh start',
        STOP:       'cd ' + ROS_CATKIN_WS + ROS_SRC_FOLDER + '/utils/alterego_battery_status/ && ./battery_monitor.sh stop',
        RESTART:    'cd ' + ROS_CATKIN_WS + ROS_SRC_FOLDER + '/utils/alterego_battery_status/ && ./battery_monitor.sh restart',
    }
};

// Function to update all launch commands with the current AlterEgoVersion
export function updateLaunchCommands() {
    // Core Nodes
    LAUNCH_COMMANDS.IMU = `roslaunch alterego_robot imu.launch AlterEgoVersion:=${AlterEgoVersion}`;
    LAUNCH_COMMANDS.DOCKING = `roslaunch alterego_docking_controller docking.launch AlterEgoVersion:=${AlterEgoVersion}`;
    LAUNCH_COMMANDS.WHEELS = `roslaunch alterego_robot wheels.launch AlterEgoVersion:=${AlterEgoVersion}`;

    // Body Movement Nodes
    LAUNCH_COMMANDS.PILOT = `roslaunch alterego_robot pilot.launch AlterEgoVersion:=${AlterEgoVersion}`;
    LAUNCH_COMMANDS.BODY_ACTIVATION = `roslaunch alterego_robot body_activation.launch AlterEgoVersion:=${AlterEgoVersion}`;
    LAUNCH_COMMANDS.BODY_MOVEMENT = `roslaunch alterego_robot body_movement.launch AlterEgoVersion:=${AlterEgoVersion}`;
    
    // Additional Nodes
    LAUNCH_COMMANDS.FACE_EXPRESSION = `roslaunch alterego_robot face_expressions.launch AlterEgoVersion:=${AlterEgoVersion}`;
    
    // Questi comandi non hanno bisogno del parametro AlterEgoVersion
    LAUNCH_COMMANDS.FACE_RECOGNITION = 'roslaunch alterego_face_recognition face_recognition.launch';
    LAUNCH_COMMANDS.FACE_TRACKING = 'roslaunch alterego_face_tracking face_tracking.launch';
    LAUNCH_COMMANDS.STT = 'roslaunch raise speech2text.launch 2>/dev/null';
    LAUNCH_COMMANDS.TTS = 'roslaunch raise text2speech.launch';
    LAUNCH_COMMANDS.NAVIGATION = 'roslaunch alterego_navigation autonomous_nav.launch';
    LAUNCH_COMMANDS.SAY_TIRED = 'roslaunch alterego_say_tired say_tired.launch';
    LAUNCH_COMMANDS.SAY_MOVE_OVER = 'roslaunch alterego_adjust_docking say_move_over.launch';
    LAUNCH_COMMANDS.BREATH = 'roslaunch alterego_rosbags_play play.launch';
    
    // Additional constants
    LAUNCH_COMMANDS.TARGET_LOC = `rostopic pub -1 /${RobotName}/target_location std_msgs/String "data: 'Mostra1'"`;
    LAUNCH_COMMANDS.DOCK_STATION = `rostopic pub -1 /${RobotName}/target_location std_msgs/String "data: 'DockStation'"`;    
    LAUNCH_COMMANDS.ADJUST_DOCKING = `/${RobotName}/adjust_docking`;
    LAUNCH_COMMANDS.STOP_BREATH = `/${RobotName}/rosbag_play`;
    
    // Stop Body Movement Nodes
    LAUNCH_COMMANDS.STOP_PILOT = {
        R_CTRL: `/${RobotName}/right/arms_compliant_control_node`,
        L_CTRL: `/${RobotName}/left/arms_compliant_control_node`,
        INBOUND: `/${RobotName}/inbound_data`,
        SOCKET: `/${RobotName}/socket`
    };
    
    LAUNCH_COMMANDS.STOP_BODY_ACTIVATION = {
        R_ARM: `/${RobotName}/right/qb_manager`,
        L_ARM: `/${RobotName}/left/qb_manager`        
    };
    
    LAUNCH_COMMANDS.STOP_BODY_MOVEMENT = {
        R_ARM: `/${RobotName}/right/arm_inv_dyn`,
        L_ARM: `/${RobotName}/left/arm_inv_dyn`,
        HEAD: `/${RobotName}/head/head_inv_kin`,
        R_MAIN: `/${RobotName}/right/arm_inv_kin_main`,
        L_MAIN: `/${RobotName}/left/arm_inv_kin_main`,
        PITCH: `/${RobotName}/pitch_correction`
    };
    
    // Stop Additional Nodes
    LAUNCH_COMMANDS.STOP_TTS = `/${RobotName}/text2speech`;
    LAUNCH_COMMANDS.STOP_STT = `/${RobotName}/speech2text`;
    LAUNCH_COMMANDS.STOP_FACE_EXPRESSION = `/${RobotName}/face_expressions`;
    LAUNCH_COMMANDS.STOP_FACE_RECOGNITION = `/${RobotName}/face_recognition`;
    LAUNCH_COMMANDS.STOP_FACE_TRACKING = `/${RobotName}/face_tracker`;
    
    // Stop Navigation Nodes
    LAUNCH_COMMANDS.STOP_NAVIGATION = {
        AMCL: `/${RobotName}/amcl`,
        LIDAR: `/${RobotName}/lidar`,
        MOVE_BASE: `/${RobotName}/move_base`,
        MAP_SERVER: `/${RobotName}/map_server`,
        MAP_SERVER_OBSTACLE: `/${RobotName}/map_server_obstacle`,
        NAVIGATION: `/${RobotName}/navigation`,
        VIS_ROBOT: `/${RobotName}/visualize_robot`
    };
    console.log('====== LAUNCH COMMANDS ======');
    console.log('Core Nodes:');
    console.log(`- IMU: ${LAUNCH_COMMANDS.IMU}`);
    console.log(`- DOCKING: ${LAUNCH_COMMANDS.DOCKING}`);
    console.log(`- WHEELS: ${LAUNCH_COMMANDS.WHEELS}`);
    console.log('Body Movement:');
    console.log(`- PILOT: ${LAUNCH_COMMANDS.PILOT}`);
    console.log(`- BODY_ACTIVATION: ${LAUNCH_COMMANDS.BODY_ACTIVATION}`);
    console.log(`- BODY_MOVEMENT: ${LAUNCH_COMMANDS.BODY_MOVEMENT}`);
    console.log(`- AlterEgo Version: ${AlterEgoVersion}`);
    console.log(`- ROBOT NAME: ${RobotName}`);
    console.log('===========================');
}

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
    },
    BATTERY_ALERT: {
        color: '#eab308',
        text: 'Battery: '
    },
    BATTERY_OK: {
        color: '#22c55e',
        text: 'Battery: '
    }
};   

// Robot configuration variables - they will be updated by loadAndApplySettings
export let CONF_FEATURES = {
    enableFaceRecognition: {
        value: false,
        label: 'Face Recognition'
    },
    enableFaceTracking: {
        value: false,
        label: 'Face Tracking'
    },
    enableAudio: {
        value: false,
        label: 'Audio'
    },
    enableVideo: {
        value: false,
        label: 'Video'
    },
    enableNavigation: {
        value: false,
        label: 'Navigation (ROS Nodes)'
    },
    enableAutoNavigation: {
        value: false,
        label: 'Autonomous Navigation (Target-DockStation)'
    },
    enableRobotBreath: {
        value: false,
        label: 'Robot Breath'
    },
};

// Initialize launch commands with default version
updateLaunchCommands();
