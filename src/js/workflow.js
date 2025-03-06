import { sendCommand,sendLocalCommand, getRobotName, initializeIMU, handleBackwardMovement, initializeSystem} from './api.js';
import { ROS_COMMANDS, LAUNCH_COMMANDS, STATE } from './constants.js';
import { updateUI } from './utils.js';

var robotName;

function updatePipelineState(ws, state, value){
    state.pipelineState = value;
            
    ws.send(JSON.stringify({
        type: 'stateUpdate',
        data: { pipelineState: value }
    }));
}

async function poweroffNUCs(ws) {

    // Power Off BASE NUC
    sendCommand('echo 111 | sudo -S poweroff', '' );

    // Power Off VISION NUC
    sendLocalCommand(`echo 111 | sudo -S poweroff`);
}

async function stopRobot() {
    // Kill everything
    sendCommand(ROS_COMMANDS.CLEANUP);
}

async function deactivateRobot() {
    // Kill everything
    sendCommand(ROS_COMMANDS.CLEANUP);
}

// --------------------- PROCEDURES --------------------------------- //
export async function activateRobotProcedures(ws) {
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.ROSCORE}`);
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.USB_DETECTOR}`);

    // Initialize IMU
    const imuInitialized = await initializeIMU(ws, robotName);
    if (!imuInitialized) {
        return false;
    }

    return true;
}

export async function standUpProcedures(ws) {

    // // Accendi il pilota
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.PILOT}`);

    // Start FACE EXPRESSION
    sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_EXPRESSION}`);

    // Start FACE TRACKING
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_RECOGNITION}`);
    await new Promise(r => setTimeout(r, 2000));

    // Start FACE TRACKING
    sendLocalCommand(`${ROS_COMMANDS.SETUP_LOCAL} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.FACE_TRACKING}`);
    await new Promise(r => setTimeout(r, 2000));
    
    // Docking Backward
    const backwardComplete =  await handleBackwardMovement(ws, robotName);
    if (!backwardComplete) {
        return false;
    }

    // Initialize system (accende ruote, attiva braccia (activation e movement))
    const systemInitialized = await initializeSystem(ws, robotName);
    if (!systemInitialized) {
        return false;
    }

    return true;
}

export async function goHomeProcedures(ws) {

    // Send Home
    //TODO

    return true;
}


// --------------------- STATE CHANGE --------------------------------- //
export async function robotPowerOnClick(ws, state) {
    
/*    robotName = await getRobotName();
    
    // Activate core nodes and moves backward
    // Activate Robot
    if (!(await activateRobotProcedures(ws))){
        return false;
    }
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.STAND_UP);
    
    // Activate all the rest of nodes 
    if (!(await standUpProcedures(ws))){
        return false;
    }         
*/    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.WORK_MODE);

    return true;
}

export async function robotPowerOffClick(ws, state) {
 /*   
    // Stop Robot
    stopRobot();

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.POWER_OFF_NUCS);
    
    // Power Off the system
    poweroffNUCs(ws);            // Power Off everything
*/
    return true;
}

export async function restartFromPauseProcedures(ws, state) {

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.WORK_MODE);

    return true;
}

export async function pauseProcedures(ws, state) {

    // Notify next workflow state
    updatePipelineState(ws, state, STATE.PAUSED);

    return true;
}

export async function dockingProcedures(ws, state) {
    
    // Kill all movement group

    // Prepare to docking
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.DOCKED);

    return true;
}

export async function endCharge(ws, state) {
    // Triggered when need_for_charge = false
    
    // Stop Robot
    stopRobot();
    
    // Notify next workflow state
    updatePipelineState(ws, state, STATE.ACTIVATE_ROBOT);

    return true;
}

export async function emergencyButtonPressed(ws, state) {
    
    // Spegni il sistema
    deactivateRobot(ws, state);

    updatePipelineState(ws, state, STATE.RECOVERY_FROM_EMERGENCY);

    // Update graphics to show both buttons
    updateUI(state);

    return true;
}