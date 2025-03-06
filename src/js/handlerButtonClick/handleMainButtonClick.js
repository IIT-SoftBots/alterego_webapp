import { pingRemoteComputer} from '../api.js';
import { STATE } from '../constants.js';
import { updateUI } from '../utils.js';
import { robotPowerOnClick, pauseProcedures, restartFromPauseProcedures } from '../workflow.js';

/**
 * Gestisce il click sul pulsante di accensione
 * Controlla la connessione e avvia/spegne il sistema
 */
export async function handleMainButtonClick(ws, state) {

    const mainBtn = document.getElementById('mainBtn');
    mainBtn.disabled = true;

    try {
        // Verifica connessione al computer base
    /*    const isRemoteComputerOnline = await pingRemoteComputer();
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
        */

        if (state.pipelineState == STATE.WORK_MODE ||
            state.pipelineState == STATE.PAUSED){

            // The system is already active --> Play / Pause

            if (state.pipelineState == STATE.WORK_MODE){
                // Clicked to Pause
                pauseProcedures(ws, state);
                state.isRunning = false;
            }
            else {
                // Clicked to Play
                restartFromPauseProcedures(ws, state);
                state.isRunning = true;
            }                    
            
            // Notify new state
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isRunning: state.isRunning }
            }));
        }
        else {
            // Clicked to Start Robot
            robotPowerOnClick(ws, state);
            state.isPowered = true;

            // Notify new state
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isPowered: state.isPowered }
            }));
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