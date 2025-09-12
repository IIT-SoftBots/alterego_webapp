import { sendCommand,sendLocalCommand, initializeIMU, handleDockingMovement, initializeSystem, waitForPowerAlertTrigger, showSyncedPopup, targetReachedCheck, checkNodeStatus, pingRemoteComputer } from './api.js';
import { batteryMonitor } from './batterymonitor.js';
import { ROS_COMMANDS, LAUNCH_COMMANDS, STATE, RobotHasKickstand, RobotHasFaceExpressions, CONF_FEATURES } from './constants.js';
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
                        robotStopClick();                        
                        state.isRunning = false;

                        ws.send(JSON.stringify({
                            type: 'stateUpdate',
                            data: { isRunning: false }
                        }));

                        updateUI();
                    }

                    if (fsmState == STATE.STOPPED && !batteryMonitor.getNeedCharge() && batteryMonitor.getShouldAutoRestart()){
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
                            fsmState != STATE.STOPPED &&
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

    // Stop Media Services
    stopMediaServices();

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
    if (RobotHasKickstand){
        const backwardComplete =  await handleDockingMovement("backward", 0.5);
        if (!backwardComplete) {
            return false;
        }
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
    if (RobotHasFaceExpressions) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_EXPRESSION}`);
    }

    // Start FACE RECOGNITION
    if (CONF_FEATURES.enableFaceRecognition.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Start FACE TRACKING
    if (CONF_FEATURES.enableFaceTracking.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Start Media Services
    await startMediaServices();

    // Start Breath Rosbag
    if (CONF_FEATURES.enableRobotBreath.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.BREATH}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Start Navigation
    if (CONF_FEATURES.enableNavigation.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.NAVIGATION}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Start Navigation Proxima
    if (CONF_FEATURES.enableNavigationProxima.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.NAVIGATION_PROXIMA.DOCKER_ROS_BRIDGE}`);
        await new Promise(r => setTimeout(r, 5000));
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.NAVIGATION}`);
        await new Promise(r => setTimeout(r, 5000));
    //    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.NAVIGATION_PROXIMA.NAV2POINTS}`);
    //    await new Promise(r => setTimeout(r, 8000));
    }

    // Send to Target Location
    if (CONF_FEATURES.enableAutoNavigation.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.TARGET_LOC}`);
        await new Promise(r => setTimeout(r, 4000));
    }

    return true;
}

export async function startAutonomousSpeech() {
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.STT}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.TTS}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function stopAutonomousSpeech(){
    
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_TTS}`);
    await new Promise(r => setTimeout(r, 2000));

    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && rosnode kill ${LAUNCH_COMMANDS.STOP_STT}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function startVideoStream(){
    const MAX_ATTEMPTS = 3;
    let attempts = 0;
    let streamStarted = false;

    const loadingPopup = Swal.fire({
        title: 'Activating video stream...',
        text: 'Please wait while the video stream is being configured.',
        timerProgressBar: true,
        allowOutsideClick: false,
        allowEscapeKey: false,
        showConfirmButton: false,
        didOpen: () => {
            Swal.showLoading();
        }
    });

    while (attempts < MAX_ATTEMPTS && !streamStarted) {
        attempts++;
        //console.log(`Attempt ${attempts} to start video stream.`);

        sendLocalCommand(`${LAUNCH_COMMANDS.VIDEO_STREAM.START}`);
        await new Promise(r => setTimeout(r, 3000)); 

        let windowExists = false;
        try {
            const response = await fetch('/grep-command-local', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ 
                    command: `${LAUNCH_COMMANDS.VIDEO_STREAM.CHECK}`
                })
            });
            
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const grepResponse = await response.json();
            
            if (grepResponse && grepResponse.output && typeof grepResponse.output === 'string' && grepResponse.output.trim() !== '') {
                //console.log("Video stream window 'send_video' found via grep. Output:", grepResponse.output);
                windowExists = true;
            } else if (grepResponse && grepResponse.error) {
                console.error("Error from grepLocalCommand:", grepResponse.error);
                windowExists = false;
            }
            else {
                console.log("Video stream window 'send_video' NOT found after start command (grep output was empty).");
                windowExists = false;
            }
        } catch (error) {
            console.error("Unexpected error checking video stream window:", error);
            windowExists = false;
        }

        if (windowExists) {
            sendLocalCommand(`${LAUNCH_COMMANDS.VIDEO_STREAM.MOVE_BG}`);
            await new Promise(r => setTimeout(r, 1000)); 
            streamStarted = true;
        } else {
            console.log(`Video stream window not found after attempt ${attempts}.`);
            if (attempts < MAX_ATTEMPTS) {
                sendLocalCommand(`${LAUNCH_COMMANDS.VIDEO_STREAM.STOP}`);
                await new Promise(r => setTimeout(r, 1000));
                console.log("Retrying...");
            }
        }
    }

    Swal.close(loadingPopup); 

    if (streamStarted) {
        await showSyncedPopup({ 
            title: 'Video Stream Active!',
            icon: 'success',
            timer: 1500,
            showConfirmButton: false
        });
        return true;
    } else {
        await showSyncedPopup({ 
            title: 'Video Stream Failed',
            text: `Could not start the video stream after ${MAX_ATTEMPTS} attempts.`,
            icon: 'error',
            showConfirmButton: true
        });
        return false;
    }
}

export async function stopVideoStream(){
    sendLocalCommand(`${LAUNCH_COMMANDS.VIDEO_STREAM.STOP}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function startAudioStream(){
    const MAX_ATTEMPTS = 3;
    let attempts = 0;
    let sendStreamStarted = false;
    let recvStreamStarted = false;

    const loadingPopup = Swal.fire({
        title: 'Activating audio streams...',
        text: 'Please wait while the audio streams are being configured.',
        timerProgressBar: true,
        allowOutsideClick: false,
        allowEscapeKey: false,
        showConfirmButton: false,
        didOpen: () => {
            Swal.showLoading();
        }
    });

    while (attempts < MAX_ATTEMPTS && (!sendStreamStarted || !recvStreamStarted)) {
        attempts++;
        //console.log(`Attempt ${attempts} to start audio streams.`);

        sendLocalCommand(`${LAUNCH_COMMANDS.AUDIO_STREAM.START}`);
        await new Promise(r => setTimeout(r, 4000));

        if (!sendStreamStarted) {
            let sendWindowExists = false;
            try {
                const grepSendResponse = await fetch('/grep-command-local', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: `${LAUNCH_COMMANDS.AUDIO_STREAM.CHECK_SEND}` })
                });
                if (!grepSendResponse.ok) throw new Error(`HTTP error for send_audio check! status: ${grepSendResponse.status}`);
                const sendResult = await grepSendResponse.json();

                if (sendResult && sendResult.output && typeof sendResult.output === 'string' && sendResult.output.trim() !== '') {
                    //console.log("Audio stream window 'send_audio' found.");
                    sendWindowExists = true;
                } else if (sendResult && sendResult.error) {
                    console.error("Error from grepLocalCommand for send_audio:", sendResult.error);
                } else {
                    console.log("Audio stream window 'send_audio' NOT found.");
                }
            } catch (error) {
                console.error("Unexpected error checking 'send_audio' window:", error);
            }

            if (sendWindowExists) {
                //console.log("'send_audio' window found. Moving to background.");
                sendLocalCommand(`${LAUNCH_COMMANDS.AUDIO_STREAM.MOVE_SEND_BG}`);
                await new Promise(r => setTimeout(r, 500));
                sendStreamStarted = true;
            } else {
                console.log(`'send_audio' window not found after attempt ${attempts}.`);
            }
        }

        if (!recvStreamStarted) {
            let recvWindowExists = false;
            try {
                const grepRecvResponse = await fetch('/grep-command-local', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: `${LAUNCH_COMMANDS.AUDIO_STREAM.CHECK_RECV}` })
                });
                if (!grepRecvResponse.ok) throw new Error(`HTTP error for recv_audio check! status: ${grepRecvResponse.status}`);
                const recvResult = await grepRecvResponse.json();

                if (recvResult && recvResult.output && typeof recvResult.output === 'string' && recvResult.output.trim() !== '') {
                    //console.log("Audio stream window 'recv_audio' found.");
                    recvWindowExists = true;
                } else if (recvResult && recvResult.error) {
                    console.error("Error from grepLocalCommand for recv_audio:", recvResult.error);
                } else {
                    console.log("Audio stream window 'recv_audio' NOT found.");
                }
            } catch (error) {
                console.error("Unexpected error checking 'recv_audio' window:", error);
            }

            if (recvWindowExists) {
                //console.log("'recv_audio' window found. Moving to background.");
                sendLocalCommand(`${LAUNCH_COMMANDS.AUDIO_STREAM.MOVE_RECV_BG}`);
                await new Promise(r => setTimeout(r, 500));
                recvStreamStarted = true;
            } else {
                console.log(`'recv_audio' window not found after attempt ${attempts}.`);
            }
        }

        if ((!sendStreamStarted || !recvStreamStarted) && attempts < MAX_ATTEMPTS) {
            console.log("One or both audio stream windows not found. Retrying...");
            sendLocalCommand(`${LAUNCH_COMMANDS.AUDIO_STREAM.STOP}`);
            await new Promise(r => setTimeout(r, 1000));
        }
    }

    Swal.close(loadingPopup);

    if (sendStreamStarted && recvStreamStarted) {
        await showSyncedPopup({
            title: 'Audio Streams Active!',
            icon: 'success',
            timer: 1500,
            showConfirmButton: false
        });
        return true;
    } else {
        let failedStreams = [];
        if (!sendStreamStarted) failedStreams.push("'send_audio'");
        if (!recvStreamStarted) failedStreams.push("'recv_audio'");
        
        await showSyncedPopup({
            title: 'Audio Stream Failed',
            text: `Could not start ${failedStreams.join(' and ')} after ${MAX_ATTEMPTS} attempts.`,
            icon: 'error',
            showConfirmButton: true
        });
        return false;
    }
}

export async function stopAudioStream(){
    sendLocalCommand(`${LAUNCH_COMMANDS.AUDIO_STREAM.STOP}`);
    await new Promise(r => setTimeout(r, 2000));

    return true;
}

export async function startMediaServices() {
    
    // Start Video Stream
    if (CONF_FEATURES.enableVideoStream.value){
        await startVideoStream();
    }

    // Start Audio Services
    if (CONF_FEATURES.enableAudioStream.value){
        await startAudioStream();
    }
    if (CONF_FEATURES.enableAutoSpeech.value){
        await startAutonomousSpeech();
    }
}

export async function stopMediaServices() {

    // Stop Video Stream
    if (CONF_FEATURES.enableVideoStream.value){
        await stopVideoStream();
        await new Promise(r => setTimeout(r, 3000));
    }

    // Stop Audio Services
    if (CONF_FEATURES.enableAudioStream.value){
        await stopAudioStream();
        await new Promise(r => setTimeout(r, 3000));
    }
    if (CONF_FEATURES.enableAutoSpeech.value){
        await stopAutonomousSpeech();
        await new Promise(r => setTimeout(r, 3000));
    }

}

export async function stopRobotMovement(){
    // Kill all movement and tracking nodes

    // Stop Pilot
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_PILOT.INBOUND} ${LAUNCH_COMMANDS.STOP_PILOT.SOCKET}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Body Movement
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.R_ARM} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.L_ARM} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.HEAD} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.R_MAIN} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.L_MAIN} ${LAUNCH_COMMANDS.STOP_BODY_MOVEMENT.PITCH}`);
    await new Promise(r => setTimeout(r, 3000));

    // Stop Body Activation
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BODY_ACTIVATION.R_ARM} ${LAUNCH_COMMANDS.STOP_BODY_ACTIVATION.L_ARM}`);
    await new Promise(r => setTimeout(r, 3000));
    
    // Stop Breath Rosbag
    if (CONF_FEATURES.enableRobotBreath.value) {
        // Stop Breath Rosbag
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BREATH}`);
        await new Promise(r => setTimeout(r, 3000));
    }

    // Stop Additional Nodes
    if (RobotHasFaceExpressions){
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_EXPRESSION}`);
        await new Promise(r => setTimeout(r, 3000));
    }

    // Stop Navigation
    if (CONF_FEATURES.enableNavigation.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_NAVIGATION.AMCL} ${LAUNCH_COMMANDS.STOP_NAVIGATION.MOVE_BASE} ${LAUNCH_COMMANDS.STOP_NAVIGATION.MAP_SERVER} ${LAUNCH_COMMANDS.STOP_NAVIGATION.MAP_SERVER_OBSTACLE} ${LAUNCH_COMMANDS.STOP_NAVIGATION.LIDAR} ${LAUNCH_COMMANDS.STOP_NAVIGATION.NAVIGATION} ${LAUNCH_COMMANDS.STOP_NAVIGATION.ROBOT_STATE_PUBLISHER} ${LAUNCH_COMMANDS.STOP_NAVIGATION.VIS_ROBOT}`);
        await new Promise(r => setTimeout(r, 4000));
    }

    // Stop Navigation Proxima
    if (CONF_FEATURES.enableNavigationProxima.value) {
        // Re-add when nav2points is used
        //sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.LIDAR} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.NAVIGATION} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.ROBOT_STATE_PUBLISHER} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.VIS_ROBOT} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.DOCKER_ROS_BRIDGE}`);
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.LIDAR} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.ROBOT_STATE_PUBLISHER} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.VIS_ROBOT} ${LAUNCH_COMMANDS.STOP_NAVIGATION_PROXIMA.DOCKER_ROS_BRIDGE}`);
        await new Promise(r => setTimeout(r, 4000));
    }

    return true;
}

export async function stopRobotWheels() {
    // Stop Wheels Control (used when no kickstand is present)

    const confirmWheels = await showSyncedPopup({ // Added await and variable to store result
        title: 'Deactivating Wheels',
        text: 'Pay attention!! Be prepared to prevent robot from falling and then Click OK to continue', // RAISE the ROBOT and then Click OK to continue.', // Modified text
        icon: 'warning',
        showConfirmButton: true, // Show confirm button
        confirmButtonText: 'OK', // Set confirm button text
        allowOutsideClick: false, // Prevent closing by clicking outside
        allowEscapeKey: false, // Prevent closing with ESC key
        showCancelButton: false // Ensure no cancel button is shown
    });       

    // Check if the user confirmed
    if (!confirmWheels) {
        console.log('Wheel deactivation cancelled by user.');
        // Optionally, show another popup or handle cancellation
        await showSyncedPopup({
            title: 'Cancelled',
            text: 'Wheel deactivation was cancelled.',
            icon: 'info',
            timer: 2000,
            showConfirmButton: false
        });
        return false; // Stop the deactivation if cancelled
    }

    // Stop Wheels
    sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_WHEELS.LQR} ${LAUNCH_COMMANDS.STOP_WHEELS.ERROR_HANDLER} ${LAUNCH_COMMANDS.STOP_WHEELS.QB_INTERFACE}`);
    await new Promise(r => setTimeout(r, 3000));

    return true;
}

export async function goHomeProcedures() {

    
    // Send Home
    
    // Navigation from current position to home room    
    sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.DOCK_STATION}`);
    await new Promise(r => setTimeout(r, 4000));

    // sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.SAY_MOVE_OVER}`);
    // await new Promise(r => setTimeout(r, 1000));

    // Alignment to charging station within fwdDistance distance (in meters)
    var targetReached = false;
    do {
        targetReached = await targetReachedCheck();
        await new Promise(r => setTimeout(r, 2000));
    }
    while (!targetReached);
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

export async function robotStopClick() {

    showLoading(true);

    // Notify next workflow state
    updatePipelineState(STATE.STOPPING);

    // Stop Face Tracking and Recognition (to do before adjust docking)
    if (CONF_FEATURES.enableFaceRecognition.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_RECOGNITION}`);
        await new Promise(r => setTimeout(r, 3000));
    }
    if (CONF_FEATURES.enableFaceTracking.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_TRACKING}`);
        await new Promise(r => setTimeout(r, 3000));
    }

    // Send Home
    var fwdDistance = 0.0; // Default value
    if (CONF_FEATURES.enableAutoNavigation.value){
        fwdDistance = await goHomeProcedures();
    }

     
    // Stop Media Services
    await stopMediaServices();

    // Kill all movement
    stopRobotMovement();

    if (RobotHasKickstand){   
        // Prepare to docking Forward
        dockingProcedures(fwdDistance);
    }
    else {
        // If no kickstand, just wait for the user to confirm wheels stop
        const stopWheels = await stopRobotWheels();
        if (!stopWheels) {
            return false;
        }
    }

    // Notify next workflow state
    updatePipelineState(STATE.STOPPED);

    showLoading(false);

    return true;
}

export async function restartFromPauseProcedures() {

    showLoading(true);

    // Start FACE RECOGNITION
    if (CONF_FEATURES.enableFaceRecognition.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Start FACE TRACKING
    if (CONF_FEATURES.enableFaceTracking.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Start Autonomous Speech
    if (CONF_FEATURES.enableAutoSpeech.value){
        startAutonomousSpeech();
    }

    // Start Breath Rosbag
    if (CONF_FEATURES.enableRobotBreath.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.BREATH}`);
        await new Promise(r => setTimeout(r, 2000));
    }

    // Notify next workflow state
    updatePipelineState(STATE.WORK_MODE);

    showLoading(false);

    return true;
}

export async function pauseProcedures() {

    showLoading(true);

    // Stop Face Tracking and Recognition (to do before adjust docking)
    if (CONF_FEATURES.enableFaceRecognition.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_RECOGNITION}`);
        await new Promise(r => setTimeout(r, 3000));
    }
    if (CONF_FEATURES.enableFaceTracking.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_FACE_TRACKING}`);
        await new Promise(r => setTimeout(r, 3000));
    }

    // Stop Breath Rosbag
    if (CONF_FEATURES.enableRobotBreath.value) {
        sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill ${LAUNCH_COMMANDS.STOP_BREATH}`);
        await new Promise(r => setTimeout(r, 2000));
    }
    
    // Stop Autonomous Speech
    if (CONF_FEATURES.enableAutoSpeech.value){
        stopAutonomousSpeech();
    }

    // Notify next workflow state
    updatePipelineState(STATE.PAUSED);

    showLoading(false);

    return true;
}

export async function dockingProcedures(maxLinDistance) {

    // Prepare to docking Forward
    const forwardComplete =  await handleDockingMovement("forward", maxLinDistance);
    if (!forwardComplete) {
        return false;
    }

    // Ask for help only if is_docked = TRUE but is_charging = FALSE ?!?
    // TODO: Check is_docked condition when having automatic docking procedure ready
    if (maxLinDistance == 0.0){
        // while (!batteryMonitor.getIsCharging()){
        //     sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && ${LAUNCH_COMMANDS.SAY_TIRED}`);
        //     await new Promise(r => setTimeout(r, 60000));   // 1 minute
        // }
    }

    // Notify next workflow state
    updatePipelineState(STATE.STOPPED);

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