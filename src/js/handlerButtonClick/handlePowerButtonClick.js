import { sendCommand, getRobotName, pingRemoteComputer, initializeIMU, handleBackwardMovement, initializeSystem} from '../api.js';
import { ROS_COMMANDS, LAUNCH_COMMANDS } from '../constants.js';

/**
 * Gestisce il click sul pulsante di accensione
 * Controlla la connessione e avvia/spegne il sistema
 */
export async function handlePowerButtonClick(ws, state) {
    try {
        // Verifica connessione al computer base
        const isRemoteComputerOnline = await pingRemoteComputer();
        if (!isRemoteComputerOnline) {
            // Mostra errore se non connesso
            const popupData = {
                title: 'ERROR',
                text: 'Base computer not connected',
                icon: 'error'
            };
            
            ws.send(JSON.stringify({
                type: 'popup',
                data: popupData
            }));
            return;
        }

        // Toggle dello stato di accensione
        state.isPowered = !state.isPowered;

        // Notifica il nuovo stato
        ws.send(JSON.stringify({
            type: 'stateUpdate',
            data: { isPowered: state.isPowered }
        }));

        if (state.isPowered) {
            // Avvia il sistema
            const robotName = await getRobotName();
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.ROSCORE}`);
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.USB_DETECTOR}`);

            // Initialize IMU
            const imuInitialized = await initializeIMU(ws, robotName);
            if (!imuInitialized) {
                return;
            }

            // // Accendi il pilota
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.PILOT}`);

            // // Procede in retro quanto basta per uscire dalla stazione di ricarica wireless
            const backwardComplete =  await handleBackwardMovement(ws, robotName);
            if (!backwardComplete) {
                return;
            }

            // Initialize system (accende ruote, attiva braccia (activation e movement))
            const systemInitialized = await initializeSystem(ws, robotName);
            if (!systemInitialized) {
                return;
            }
        } else {
            // Spegni il sistema
            sendCommand(ROS_COMMANDS.CLEANUP);
            state.isRunning = false;
            
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isRunning: false }
            }));
        }

    } catch (error) {
        // Gestione errori
        console.error('Power button error:', error);
        state.isPowered = !state.isPowered;  // Ripristina stato precedente
        
        // Mostra errore
        ws.send(JSON.stringify({
            type: 'popup',
            data: {
                title: 'Error',
                text: error.message,
                icon: 'error'
            }
        }));
    }
}

