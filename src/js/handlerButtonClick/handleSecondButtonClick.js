import { showSyncedPopup } from '../api.js';
import { batteryMonitor } from '../batterymonitor.js';
import { STATE } from '../constants.js';
import { updateUI } from '../utils.js';
import { robotHomeClick, robotPowerOffClick } from '../workflow.js';
import { logStartupEvent, newStartupAttemptId } from '../startupLog.js';
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
                title: 'Spegni il robot',
                text: "Il robot verrà disattivato e i computer si spegneranno. Vuoi continuare?",
                icon: 'warning',
                showCancelButton: true, cancelButtonText: 'Annulla',
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'Sì, spegni'
            });
            
            const shutdownId = newStartupAttemptId();
            logStartupEvent(shutdownId, 'shutdown.confirm', { outcome: warnAnsw ? 'accepted' : 'cancelled', pipelineState: state.pipelineState });
            if (!warnAnsw) return;

            // Switch Everything Off
            robotPowerOffClick(ws, state, true, shutdownId);
            state.isPowered = false;
            
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isPowered: false }
            }));
        }
        else {
            // Clicked to Go Home
            const warnAnsw = await showSyncedPopup(ws, {
                title: 'Torna in base',
                text: "Il robot andrà da solo verso la base di ricarica. Vuoi continuare?",
                icon: 'warning',
                showCancelButton: true, cancelButtonText: 'Annulla',
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'Sì, torna in base'
            });
            
            if (!warnAnsw) return;

            // Homing
            batteryMonitor.setShouldAutoRestart(false);
            robotHomeClick(ws, state, robotName);
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