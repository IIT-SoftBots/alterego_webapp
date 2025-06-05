import { sendCommand,sendLocalCommand, initializeIMU, handleDockingMovement, initializeSystem, waitForPowerAlertTrigger, showSyncedPopup, targetReachedCheck, checkNodeStatus, pingRemoteComputer} from './api.js';
import { batteryMonitor } from './batterymonitor.js';
import { ROS_COMMANDS, LAUNCH_COMMANDS, STATE } from './constants.js';
import { state, ws } from './main.js';
import { showLoading, updateUI } from './utils.js';

var batteryInterval;

function updatePipelineState(value){
    state.pipelineState = value;
            
    ws.send(JSON.stringify({
        type: 'stateUpdate',
        data: { pipelineState: value }
    }));

    batteryMonitor.updateFSMState(state.pipelineState);
}

async function poweroffNUCs() {

    // Power Off BASE NUC
    sendCommand('echo 111 | sudo -S poweroff', '' );
    await new Promise(r => setTimeout(r, 2000));
    
    // Power Off VISION NUC
    sendLocalCommand(`echo 111 | sudo -S poweroff`, '');
}

async function startPowerMonitor(){
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
                        robotHomeClick();                        
                        state.isRunning = false;

                        ws.send(JSON.stringify({
                            type: 'stateUpdate',
                            data: { isRunning: false }
                        }));

                        updateUI();
                    }

                    if (fsmState == STATE.DOCKED && !batteryMonitor.getNeedCharge() && batteryMonitor.getShouldAutoRestart()){
                        // Triggered when need_for_charge = false
    
                        endChargeProcedures();
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
                            emergencyButtonPressed();
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
async function stopRobot() {

    stopPowerMonitor(); // Need max. 1500 ms to close

    // Kill everything
    sendCommand(ROS_COMMANDS.CLEANUP);

    // Popup - Wait nodes have been killed
    await showSyncedPopup({
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

async function deactivateRobot() {

    stopPowerMonitor(); // Need max. 1500 ms to close

    // Kill everything
    sendCommand(ROS_COMMANDS.CLEANUP);
    
    // Popup - Wait nodes have been killed
    await showSyncedPopup({
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

async function checkForPowerOn(){

    // Wait for Power Alert Trigger before moving
    const powerIsOn = await waitForPowerAlertTrigger(true);
    if (!powerIsOn) {
        return false;
    }

    return true;
}

export async function activateRobotProcedures() {
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.ROSCORE}`);
    await new Promise(r => setTimeout(r, 2000));
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.USB_DETECTOR}`);
    await new Promise(r => setTimeout(r, 2000));
    sendCommand(ROS_COMMANDS.CLEAR_LOG);
    await new Promise(r => setTimeout(r, 2000));
    
    // Initialize IMU
    const imuInitialized = await initializeIMU();
    if (!imuInitialized) {
        sendCommand(ROS_COMMANDS.CLEAR_LOG);
        await new Promise(r => setTimeout(r, 2000));
        
        // Try once again
        const imuInitRetry = await initializeIMU();
        if (!imuInitRetry){
            return false;
        }
    }

    // Check For Power ON
    const checkPowerON = await checkForPowerOn();
    if (!checkPowerON) {
        return false;
    }

    // Docking Backward
    const backwardComplete =  await handleDockingMovement("backward", 0.5);
    if (!backwardComplete) {
        return false;
    }

    return true;
}

export async function standUpProcedures() {

    // Initialize system (activates wheels and arms (activation e movement))
    const systemInitialized = await initializeSystem();
    if (!systemInitialized) {
        return false;
    }

    // Activate all remaining additional nodes

    // Start Pilot
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.PILOT}`);

    // Start FACE EXPRESSION
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_EXPRESSION}`);

    // Start FACE TRACKING and FACE RECOGNITION

    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));

    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start Audio Services
    startAudio();

    // Start Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.BREATH}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start Navigation
    // sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.NAVIGATION}`);
    // await new Promise(r => setTimeout(r, 2000));

    // Send to Target Location
    // sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.TARGET_LOC}`);
    // await new Promise(r => setTimeout(r, 4000));

    return true;
}

export async function startAudio() {
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.STT}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.TTS}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function stopAudio(){
    
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_TTS}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_STT}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function stopRobotMovement(){
    // Kill all movement and tracking nodes

    // Stop Pilot
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_PILOT.R_CTRL} ${LAUNCH_COMMANDS.STOP_PILOT.L_CTRL} ${LAUNCH_COMMANDS.STOP_PILOT.INBOUND} ${LAUNCH_COMMANDS.STOP_PILOT.SOCKET}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Body Movement
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.R_ARM} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.L_ARM} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.HEAD} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.R_MAIN} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.L_MAIN} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.PITCH}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Body Activation
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BODY_ACTIVATION.R_ARM} ${LAUNCH_COMMANDS.STOP_BODY_ACTIVATION.L_ARM}`);
    await new Promise(r => setTimeout(r, 3000));
    
    // Stop Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BREATH}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Audio Services
    stopAudio();
    await new Promise(r => setTimeout(r, 3000));

    // Stop Additional Nodes
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_EXPRESSION}`);
    await new Promise(r => setTimeout(r, 3000));
  
    // Stop Navigation
    // sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_NAVIGATION.AMCL} ${LAUNCH_COMMANDS.STOP_NAVIGATION.MOVE_BASE} ${LAUNCH_COMMANDS.STOP_NAVIGATION.MAP_SERVER} ${LAUNCH_COMMANDS.STOP_NAVIGATION.MAP_SERVER_OBSTACLE} ${LAUNCH_COMMANDS.STOP_NAVIGATION.LIDAR} ${LAUNCH_COMMANDS.STOP_NAVIGATION.NAVIGATION} ${LAUNCH_COMMANDS.STOP_NAVIGATION.VIS_ROBOT}`);
    // await new Promise(r => setTimeout(r, 4000));

    return true;
}

export async function goHomeProcedures() {

    // Send Home

    // Stop Face Tracking and Recognition (to do before adjust docking)
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 3000));
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 3000));

    
    // Navigation from current position to home room
    // sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.DOCK_STATION}`);  // Doubled to be sure
    // await new Promise(r => setTimeout(r, 4000));

    // sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.SAY_MOVE_OVER}`);
    // await new Promise(r => setTimeout(r, 1000));

    // Alignment to charging station within fwdDistance distance (in meters)
    // var targetReached = false;
    // do {
    //     targetReached = await targetReachedCheck();
    //     await new Promise(r => setTimeout(r, 2000));
    // }
    // while (!targetReached);
/*
    var approachActive = true;
    do {
        approachActive = await checkNodeStatus(`${LAUNCH_COMMANDS.ADJUST_DOCKING}`);
        await new Promise(r => setTimeout(r, 2000));
    } while(approachActive);
*/    

    var fwdDistance = 0.0;      // Override previous valutation
    
    return fwdDistance;
}

// --------------------- STATE CHANGE --------------------------------- //

async function stopRobotToPowerOff() {
    
    // Wait for Power Alert Trigger
    if (batteryMonitor.timerIsSet()){
        const powerIsOff = await waitForPowerAlertTrigger(false);

        /*if (!powerIsOff) {
            // Error on trigger or battery timer or ROS not set
            // Go ahead the same to power off the system        
        }*/
    }
    
    if (state.pipelineState != STATE.RECOVERY_FROM_EMERGENCY){
        stopRobot();
    }

    return true;
}

export async function robotPowerOnClick() {

    showLoading(true);
   
    // Initialize Timer to monitor power issues and battery level
    startPowerMonitor();

    // Notify next workflow state
    updatePipelineState(STATE.ACTIVATE_ROBOT);
    
    // Activate Robot Core nodes and moves backward
    if (!(await activateRobotProcedures())){
        return false;
    }
    
    // Notify next workflow state
    updatePipelineState(STATE.STAND_UP);
    
    // Activate all the rest of nodes 
    if (!(await standUpProcedures())){
        return false;
    }         
    
    // Notify next workflow state
    updatePipelineState(STATE.WORK_MODE);

    showLoading(false);

    return true;
}

export async function robotPowerOffClick() {

    // Stop Robot to Power Off
    const stopRobotPowerOff = await stopRobotToPowerOff();
    if (!stopRobotPowerOff) {
        return false;
    }
    
    // Notify next workflow state
    updatePipelineState(STATE.POWER_OFF_NUCS);
    
    // Power Off the system
    poweroffNUCs();            // Power Off everything

    // Notify next workflow state (useless, everything is off)
    updatePipelineState(STATE.INIT);

    return true;
}

export async function robotHomeClick() {

    showLoading(true);

    // Send Home
    const fwdDistance = await goHomeProcedures();

    // Dock to charging station
    dockingProcedures(fwdDistance);

    showLoading(false);

}

export async function restartFromPauseProcedures() {

    showLoading(true);

    // Start FACE RECOGNITION and TRACKING
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start Audio Services
    startAudio();

    // Start Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.BREATH}`);

    // Notify next workflow state
    updatePipelineState(STATE.WORK_MODE);

    showLoading(false);

    return true;
}

export async function pauseProcedures() {

    showLoading(true);

    // Stop Face Recognition
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));

    // Stop Breath Rosbag
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BREATH}`);
    await new Promise(r => setTimeout(r, 2000));

    // Stop Audio Services
    stopAudio();
    await new Promise(r => setTimeout(r, 2000));

    // Notify next workflow state
    updatePipelineState(STATE.PAUSED);

    showLoading(false);

    return true;
}

export async function dockingProcedures(maxLinDistance) {
    
    // Kill all movement
    stopRobotMovement();

    // Prepare to docking Forward
    const forwardComplete =  await handleDockingMovement("forward", maxLinDistance);
    if (!forwardComplete) {
        return false;
    }

    // Ask for help only if is_docked = TRUE but is_charging = FALSE ?!?
    // TODO: Check is_docked condition when having automatic docking procedure ready
    if (maxLinDistance == 0.0){
        while (!batteryMonitor.getIsCharging()){
            sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.SAY_TIRED}`);
            await new Promise(r => setTimeout(r, 60000));   // 1 minute
        }
    }

    // Notify next workflow state
    updatePipelineState(STATE.DOCKED);

    return true;
}

// ----------- ASYNC ROUTINES WITH STATE CHANGE -------------------- //
export async function endChargeProcedures() {
    
    // Stop Robot
    stopRobot();
    
    // Notify next workflow state
    updatePipelineState(STATE.RESTART_AUTO);

    // Reload the web page and force clear the cache
    window.location.reload(true);

    return true;
}

export async function restartAuto() {

    // Popup - Info to auto restart
    await showSyncedPopup({
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
    
    robotPowerOnClick();   //Restart
    state.isPowered = true;

    // Notify new state
    ws.send(JSON.stringify({
        type: 'stateUpdate',
        data: { isPowered: state.isPowered }
    }));

    // Update graphics to show both buttons
    updateUI();

    return true;
}

export async function emergencyButtonPressed() {
    // Triggered when power_alert = true

    // Deactivate Robot
    deactivateRobot();

    // Notify next workflow state
    updatePipelineState(STATE.RECOVERY_FROM_EMERGENCY);

    // Update graphics to show both buttons
    updateUI();

    return true;
}

export async function overrideInitRobotState(){

    const isRemoteComputerOnline = await pingRemoteComputer();
    if (isRemoteComputerOnline) {
        sendCommand(ROS_COMMANDS.CLEANUP);
        sendCommand(ROS_COMMANDS.CLEAR_LOG);
    }

    await showSyncedPopup({
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