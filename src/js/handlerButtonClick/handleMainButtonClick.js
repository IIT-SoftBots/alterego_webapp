import { pingRemoteComputer, showSyncedPopup} from '../api.js';
import { STATE } from '../constants.js';
import { updateUI } from '../utils.js';
import { robotPowerOnClick, pauseProcedures, restartFromPauseProcedures, endChargeProcedures } from '../workflow.js';

/**
 * Gestisce il click sul pulsante di accensione
 * Controlla la connessione e avvia/spegne il sistema
 */
export async function handleMainButtonClick(ws, state, robotName) {

    const mainBtn = document.getElementById('mainBtn');
    mainBtn.disabled = true;

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
        console.log('Main button clicked in INIT, DOCKED or RECOVERY state');

                                 

        if (state.pipelineState == STATE.WORK_MODE ||
            state.pipelineState == STATE.PAUSED){
            console.log('work mode or paused');

            // The system is already active --> Play / Pause

            if (state.pipelineState == STATE.WORK_MODE){
                // Clicked to Pause
                pauseProcedures(ws, state, robotName);
                state.isRunning = false;
            }
            else {
                // Clicked to Play
                restartFromPauseProcedures(ws, state, robotName);
                state.isRunning = true;
            }                    
            
            // Notify new state
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isRunning: state.isRunning }
            }));
        }
        //print state
        console.log(state.pipelineState);

        if (state.pipelineState == STATE.INIT ||
            state.pipelineState == STATE.DOCKED ||
            state.pipelineState == STATE.RECOVERY_FROM_EMERGENCY){
            console.log('Main button clicked in INIT, DOCKED or RECOVERY state');

            // First popup - Global warning
            const warnAnsw = await showSyncedPopup(ws, {
                title: 'Start Robot',
                text: "The system will move back and activate the robot now. Are you sure?",
                icon: 'warning',
                showCancelButton: true,
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'OK, Activate Robot'
            });
            
            if (!warnAnsw) return false;

            if (state.pipelineState == STATE.DOCKED) {  // In this case, the robot is still active
                endChargeProcedures(ws, state);
            }
            else {
                // Clicked to Start Robot
                robotPowerOnClick(ws, state, robotName);
                state.isPowered = true;

                // Notify new state
                ws.send(JSON.stringify({
                    type: 'stateUpdate',
                    data: { isPowered: state.isPowered }
                }));
            }            
        }

    } catch (error) {
        console.error('Main button error:', error);
        // No need to restore state, since change happens after procedures    
        ws.send(JSON.stringify({
            type: 'popup',
            data: {
                title: 'Error',
                text: error.message,
                icon: 'error'
            }
        }));
    }
    finally {

        updateUI(state);
        
        mainBtn.disabled = false;
    }
}