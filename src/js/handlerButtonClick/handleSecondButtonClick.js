import { showSyncedPopup } from '../api.js';
import { STATE } from '../constants.js';
import { updateUI } from '../utils.js';
import { dockingProcedures, goHomeProcedures, robotPowerOffClick } from '../workflow.js';
/**
 * Gestisce il click sul pulsante Home
 * Avvia il movimento verso la posizione home
 */
export async function handleSecondButtonClick(ws, state) {

    const secondBtn = document.getElementById('secondBtn');
    secondBtn.disabled = true;

    try {
     
        if (state.pipelineState == STATE.INIT ||
            state.pipelineState == STATE.DOCKED ||
            state.pipelineState == STATE.RECOVERY_FROM_EMERGENCY){
            // Clicked to Power Off Robot
            showSyncedPopup(ws, {
                title: 'Power Off',
                text: 'Deactivating the Robot...',
                icon: 'info',
                timer: 2000,
                timerProgressBar: true,
                showConfirmButton: false
            });

            // Switch Everything Off
            robotPowerOffClick(ws, state);
            state.isPowered = false;
            
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isPowered: false }
            }));
        }
        else {
            // Clicked to Go Home
            showSyncedPopup(ws, {
                title: 'Home Position',
                text: 'Moving to home position...',
                icon: 'info',
                timer: 2000,
                timerProgressBar: true,
                showConfirmButton: false
            });

            // Send Home
            goHomeProcedures(ws);

            dockingProcedures(ws, state);
            state.isRunning = false;

            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isRunning: false }
            }));
        }

    } catch (error) {
        console.error('Second button error:', error);
        // No need to restore state, since change happens after procedures
        Swal.fire('Error', error.message, 'error');
    }  
    finally {

        updateUI(state);

        secondBtn.disabled = false;
    }    
}