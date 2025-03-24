import { showSyncedPopup } from '../api.js';
import { batteryMonitor } from '../batterymonitor.js';
import { STATE } from '../constants.js';
import { updateUI } from '../utils.js';
import { robotHomeClick, robotPowerOffClick } from '../workflow.js';
/**
 * Gestisce il click sul pulsante Home
 * Avvia il movimento verso la posizione home
 */
export async function handleSecondButtonClick(ws, state, robotName) {

    const secondBtn = document.getElementById('secondBtn');
    secondBtn.disabled = true;

    try {
     
        if (state.pipelineState == STATE.INIT ||
            state.pipelineState == STATE.DOCKED ||
            state.pipelineState == STATE.RECOVERY_FROM_EMERGENCY){

            // First popup - Global warning
            const warnAnsw = await showSyncedPopup(ws, {
                title: 'Power Off',
                text: "The system will deactivate the robot and power off now. Are you sure?",
                icon: 'warning',
                showCancelButton: true,
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'OK, Power Off'
            });
            
            if (!warnAnsw) return;

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
            const warnAnsw = await showSyncedPopup(ws, {
                title: 'Home Position',
                text: "The system will navigate towards its home now. Are you sure?",
                icon: 'warning',
                showCancelButton: true,
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'OK, go Home'
            });
            
            if (!warnAnsw) return;

            // Homing
            batteryMonitor.setShouldAutoRestart(false);
            robotHomeClick(ws, state);
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