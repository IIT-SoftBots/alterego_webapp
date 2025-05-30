import { sendCommand,sendLocalCommand, getRobotName, initializeIMU, handleDockingMovement, initializeSystem, waitForPowerAlertTrigger, showSyncedPopup, targetReachedCheck, checkNodeStatus, pingRemoteComputer} from './api.js';
import { batteryMonitor } from './batterymonitor.js';
import { ROS_COMMANDS, LAUNCH_COMMANDS, STATE } from './constants.js';
import { showLoading, updateUI } from './utils.js';

var batteryInterval;

function updatePipelineState(ws, state, value){
    state.pipelineState = value;
            
    ws.send(JSON.stringify({
        type: 'stateUpdate',
        data: { pipelineState: value }
    }));

    batteryMonitor.updateFSMState(state.pipelineState);
}

async function poweroffNUCs(ws) {

    // Power Off BASE NUC
    sendCommand('echo 111 | sudo -S poweroff', '' );
    await new Promise(r => setTimeout(r, 2000));
    
    // Power Off VISION NUC
    sendLocalCommand(`echo 111 | sudo -S poweroff`, '');
}

async function startPowerMonitor(ws, state, robotName){
    const POLLING_INTERVAL = 1500;

    batteryInterval = setInterval(async () => {
        try {
            
            if (batteryMonitor.timerIsSet()){
                // Battery topic is monitored
       
                const fsmState = batteryMonitor.getState(); // Need something that is updated on the background
                //console.log("FSM PIPELINE STATE: " + fsmState);
                
                if (batteryMonitor.checkNeedChargeTrigger()) {
                    // Detect and handle change in power alert 
                    if (fsmState == STATE.WORK_MODE && batteryMonitor.getNeedCharge()){
                        // Triggered when need_for_charge = true

                        // Send Home to Charge
                        batteryMonitor.setShouldAutoRestart(true);
                        robotHomeClick(ws, state, robotName);                        
                        state.isRunning = false;

                        ws.send(JSON.stringify({
                            type: 'stateUpdate',
                            data: { isRunning: false }
                        }));

                        updateUI(state);
                    }

                    if (fsmState == STATE.DOCKED && !batteryMonitor.getNeedCharge() && batteryMonitor.getShouldAutoRestart()){
                        // Triggered when need_for_charge = false
    
                        endChargeProcedures(ws, state);
                    }
                }

                // Always check for Emergency Button Pressed
                if (batteryMonitor.checkPowerAlertTrigger()) {
                    // Detect and handle change in power alert 
                    if (batteryMonitor.getPowerAlert()){
                        // Trigger from False to True
                        
                        console.log("Emergency Button Pressed to remove power");
                        
                        if (fsmState != STATE.INIT &&
                            fsmState != STATE.DOCKED &&
                            fsmState != STATE.RECOVERY_FROM_EMERGENCY){
                            // Emergency Button must deactivate the robot that is working
                            emergencyButtonPressed(ws, state);
                        }
                        
                    }
                    else {
                        // Trigger from True to False
                        console.log("Emergency Button Pressed to give power");
                        
                    }
                }
            }       

        } catch (error) {
            console.error('Error in global timer:', error);
        }
    }, POLLING_INTERVAL);
}

function stopPowerMonitor(){
    if (batteryInterval) {
        clearInterval(batteryInterval);
    }
}

// --------------------- PROCEDURES --------------------------------- //
async function stopRobot(ws) {

    stopPowerMonitor(); // Need max. 1500 ms to close

    // Kill everything
    sendCommand(ROS_COMMANDS.CLEANUP);

    // Popup - Wait nodes have been killed
    await showSyncedPopup(ws, {
        title: 'Stop Robot',
        text: "Robot is deactivating. Please wait...",
        icon: 'warning',
        showCancelButton: false,
        allowOutsideClick: false,
        allowEscapeKey: false,
        timer: 18000,           // 18 sec. to be sure nodes have been killed
        timerProgressBar: true,
        showConfirmButton: false
    });  

    sendCommand(ROS_COMMANDS.CLEAR_LOG);
}

async function deactivateRobot(ws) {

    stopPowerMonitor(); // Need max. 1500 ms to close

    // Kill everything
    sendCommand(ROS_COMMANDS.CLEANUP);
    
    // Popup - Wait nodes have been killed
    await showSyncedPopup(ws, {
        title: 'Stop Robot',
        text: "Robot is deactivating. Please wait...",
        icon: 'warning',
        showCancelButton: false,
        allowOutsideClick: false,
        allowEscapeKey: false,
        timer: 18000,           // 18 sec. to be sure nodes have been killed
        timerProgressBar: true,
        showConfirmButton: false
    });  

    sendCommand(ROS_COMMANDS.CLEAR_LOG);
}

async function checkForPowerOn(ws, robotName){

    // Wait for Power Alert Trigger before moving
    const powerIsOn = await waitForPowerAlertTrigger(ws, true);
    if (!powerIsOn) {
        return false;
    }

    return true;
}

export async function activateRobotProcedures(ws, robotName) {
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.ROSCORE}`);
    await new Promise(r => setTimeout(r, 2000));
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.USB_DETECTOR}`);
    await new Promise(r => setTimeout(r, 2000));
    sendCommand(ROS_COMMANDS.CLEAR_LOG);
    await new Promise(r => setTimeout(r, 2000));
    
    // Initialize IMU
    const imuInitialized = await initializeIMU(ws, robotName);
    if (!imuInitialized) {
        sendCommand(ROS_COMMANDS.CLEAR_LOG);
        await new Promise(r => setTimeout(r, 2000));
        
        // Try once again
        const imuInitRetry = await initializeIMU(ws, robotName);
        if (!imuInitRetry){
            return false;
        }
    }

    // Check For Power ON
    const checkPowerON = await checkForPowerOn(ws, robotName);
    if (!checkPowerON) {
        return false;
    }

    // Docking Backward
    const backwardComplete =  await handleDockingMovement(ws, robotName, "backward", 0.5);
    if (!backwardComplete) {
        return false;
    }

    return true;
}

export async function standUpProcedures(ws, robotName) {

    // Initialize system (activates wheels and arms (activation e movement))
    const systemInitialized = await initializeSystem(ws, robotName);
    if (!systemInitialized) {
        return false;
    }

    // Activate all remaining additional nodes

    // Start Pilot
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.PILOT}`);

    // Start FACE EXPRESSION
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_EXPRESSION}`);

    // Start FACE TRACKING and FACE RECOGNITION

    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));

    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start Audio Services
    startAudio(ws, robotName);

    // Start Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BREATH}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start Navigation
    // sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.NAVIGATION}`);
    // await new Promise(r => setTimeout(r, 2000));

    // Send to Target Location
    // sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rostopic pub -1 /${robotName}/target_location std_msgs/String "data: '${LAUNCH_COMMANDS.TARGET_LOC}'"`);
    // await new Promise(r => setTimeout(r, 4000));

    return true;
}

export async function startAudio(ws, robotName) {
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.STT}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.TTS}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function stopAudio(ws, robotName){
  
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_TTS}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_STT}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function stopRobotMovement(ws, robotName){
    // Kill all movement and tracking nodes

    // Stop Pilot
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_PILOT.R_CTRL} /${robotName}${LAUNCH_COMMANDS.STOP_PILOT.L_CTRL} /${robotName}${LAUNCH_COMMANDS.STOP_PILOT.INBOUND} /${robotName}${LAUNCH_COMMANDS.STOP_PILOT.SOCKET}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Body Movement
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.R_ARM} /${robotName}${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.L_ARM} /${robotName}${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.HEAD} /${robotName}${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.R_MAIN} /${robotName}${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.L_MAIN} /${robotName}${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.PITCH}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Body Activation
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_BODY_ACTIVATION.R_ARM} /${robotName}${LAUNCH_COMMANDS.STOP_BODY_ACTIVATION.L_ARM}`);
    await new Promise(r => setTimeout(r, 3000));
    
    // Stop Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_BREATH}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Audio Services
    stopAudio(ws, robotName);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Additional Nodes
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_FACE_EXPRESSION}`);
    await new Promise(r => setTimeout(r, 3000));
  
    // Stop Navigation
    // sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.AMCL} /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.MOVE_BASE} /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.MAP_SERVER} /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.MAP_SERVER_OBSTACLE} /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.LIDAR} /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.NAVIGATION} /${robotName}${LAUNCH_COMMANDS.STOP_NAVIGATION.VIS_ROBOT}`);
    // await new Promise(r => setTimeout(r, 4000));

    return true;
}

export async function goHomeProcedures(ws, robotName) {

    // Send Home

    // Stop Face Tracking and Recognition (to do before adjust docking)
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 3000));
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 3000));

    
    // Navigation from current position to home room
    // sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rostopic pub -1 /${robotName}/target_location std_msgs/String "data: '${LAUNCH_COMMANDS.DOCK_STATION}'"`);  // Doubled to be sure
    // await new Promise(r => setTimeout(r, 4000));

    // sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.SAY_MOVE_OVER}`);
    // await new Promise(r => setTimeout(r, 1000));

    // Alignment to charging station within fwdDistance distance (in meters)
    // var targetReached = false;
    // do {
    //     targetReached = await targetReachedCheck(robotName);
    //     await new Promise(r => setTimeout(r, 2000));
    // }
    // while (!targetReached);
/*
    var approachActive = true;
    do {
        approachActive = await checkNodeStatus(`/${robotName}${LAUNCH_COMMANDS.ADJUST_DOCKING}`);
        await new Promise(r => setTimeout(r, 2000));
    } while(approachActive);
*/    

    var fwdDistance = 0.0;      // Override previous valutation
    
    return fwdDistance;
}

// --------------------- STATE CHANGE --------------------------------- //

async function stopRobotToPowerOff(ws, state) {
    
    // Wait for Power Alert Trigger
    if (batteryMonitor.timerIsSet()){
        const powerIsOff = await waitForPowerAlertTrigger(ws, false);

        /*if (!powerIsOff) {
            // Error on trigger or battery timer or ROS not set
            // Go ahead the same to power off the system        
        }*/
    }
    
    if (state.pipelineState != STATE.RECOVERY_FROM_EMERGENCY){
        stopRobot(ws);
    }

    return true;
}

export async function robotPowerOnClick(ws, state, robotName) {

    robotName = await getRobotName();   // Doubled to be sure

    showLoading(true);
   
    // Initialize Timer to monitor power issues and battery level
    startPowerMonitor(ws, state, robotName);

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.ACTIVATE_ROBOT);
    
    // Activate Robot Core nodes and moves backward
    if (!(await activateRobotProcedures(ws, robotName))){
        return false;
    }
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.STAND_UP);
    
    // Activate all the rest of nodes 
    if (!(await standUpProcedures(ws, robotName))){
        return false;
    }         
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.WORK_MODE);

    showLoading(false);

    return true;
}

export async function robotPowerOffClick(ws, state) {

    // Stop Robot to Power Off
    const stopRobotPowerOff = await stopRobotToPowerOff(ws, state);
    if (!stopRobotPowerOff) {
        return false;
    }
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.POWER_OFF_NUCS);
    
    // Power Off the system
    poweroffNUCs(ws);            // Power Off everything

    // Notify next workflow state (useless, everything is off)
    updatePipelineState(ws, state, STATE.INIT);

    return true;
}

export async function robotHomeClick(ws, state, robotName) {

    showLoading(true);

    // Send Home
    const fwdDistance = await goHomeProcedures(ws, robotName);

    // Dock to charging station
    dockingProcedures(ws, state, fwdDistance, robotName);

    showLoading(false);

}

export async function restartFromPauseProcedures(ws, state, robotName) {

    showLoading(true);

    // Start FACE RECOGNITION and TRACKING
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));

    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start Audio Services
    startAudio(ws, robotName);

    // Start Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BREATH}`);

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.WORK_MODE);

    showLoading(false);

    return true;
}

export async function pauseProcedures(ws, state, robotName) {

    showLoading(true);

    // Stop Face Recognition
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));

    // Stop Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && rosnode kill /${robotName}${LAUNCH_COMMANDS.STOP_BREATH}`);
    await new Promise(r => setTimeout(r, 2000));

    // Stop Audio Services
    stopAudio(ws, robotName);
    await new Promise(r => setTimeout(r, 2000));

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.PAUSED);

    showLoading(false);

    return true;
}

export async function dockingProcedures(ws, state, maxLinDistance, robotName) {
    
    // Kill all movement
    stopRobotMovement(ws, robotName);

    // Prepare to docking Forward
    const forwardComplete =  await handleDockingMovement(ws, robotName, "forward", maxLinDistance);
    if (!forwardComplete) {
        return false;
    }

    // Ask for help only if is_docked = TRUE but is_charging = FALSE ?!?
    // TODO: Check is_docked condition when having automatic docking procedure ready
    if (maxLinDistance == 0.0){
        while (!batteryMonitor.getIsCharging()){
            sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.SAY_TIRED}`);
            await new Promise(r => setTimeout(r, 60000));   // 1 minute
        }
    }

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.DOCKED);

    return true;
}

// ----------- ASYNC ROUTINES WITH STATE CHANGE -------------------- //
export async function endChargeProcedures(ws, state) {
    
    // Stop Robot
    stopRobot(ws);
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.RESTART_AUTO);

    // Reload the web page and force clear the cache
    window.location.reload(true);

    return true;
}

export async function restartAuto(ws, state, robotName) {

    // Popup - Info to auto restart
    await showSyncedPopup(ws, {
        title: 'Restart Work',
        text: "Pay attention! In a few time the system will move back and activate the robot",
        icon: 'warning',
        showCancelButton: false,
        allowOutsideClick: false,
        allowEscapeKey: false,
        timer: 10000,
        timerProgressBar: true,
        showConfirmButton: false
    });
    
    robotPowerOnClick(ws, state, robotName);   //Restart
    state.isPowered = true;

    // Notify new state
    ws.send(JSON.stringify({
        type: 'stateUpdate',
        data: { isPowered: state.isPowered }
    }));

    // Update graphics to show both buttons
    updateUI(state);

    return true;
}

export async function emergencyButtonPressed(ws, state) {
    // Triggered when power_alert = true

    // Deactivate Robot
    deactivateRobot(ws);

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.RECOVERY_FROM_EMERGENCY);

    // Update graphics to show both buttons
    updateUI(state);

    return true;
}

export async function overrideInitRobotState(ws, state){

    const isRemoteComputerOnline = await pingRemoteComputer();
    if (isRemoteComputerOnline) {
        sendCommand(ROS_COMMANDS.CLEANUP);
        sendCommand(ROS_COMMANDS.CLEAR_LOG);
    }

    await showSyncedPopup(ws, {
        title: 'Initialization',
        text: 'State is initializing. Please wait...',
        icon: 'warning',
        showCancelButton: false,
        allowOutsideClick: false,
        allowEscapeKey: false,
        timer: 5000,
        timerProgressBar: true,
        showConfirmButton: false
    });    

    state.pipelineState = STATE.INIT;
    state.isPowered = false;
        
    ws.send(JSON.stringify({
        type: 'stateUpdate',
        data: { pipelineState: state.pipelineState,
                isPowered: state.isPowered
        }
    })); 
}