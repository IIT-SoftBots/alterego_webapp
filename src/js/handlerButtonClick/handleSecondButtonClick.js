import { showSyncedPopup } from '../api.js';
import { batteryMonitor } from '../batterymonitor.js';
import { CONF_FEATURES, STATE } from '../constants.js';
import { state, ws } from '../main.js';
import { updateUI } from '../utils.js';
import { robotStopClick, robotPowerOffClick } from '../workflow.js';
/**
 * Gestisce il click sul pulsante Home
 * Avvia il movimento verso la posizione home
 */
export async function handleSecondButtonClick() {

    const secondBtn = document.getElementById('secondBtn');
    secondBtn.disabled = true;

    try {
     
        if (state.pipelineState == STATE.INIT ||
            state.pipelineState == STATE.STOPPED ||
            state.pipelineState == STATE.RECOVERY_FROM_EMERGENCY){

            // First popup - Global warning
            const warnAnsw = await showSyncedPopup({
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
            robotPowerOffClick();
            state.isPowered = false;
            
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { isPowered: false }
            }));
        }
        else {
            if (CONF_FEATURES.enableAutoNavigation.value) {
                // If auto navigation is enabled, show 'Home' button
        
                // Clicked to Go Home
                const warnAnsw = await showSyncedPopup({
                    title: 'Home Position',
                    text: "The system will navigate towards its home now. Are you sure?",
                    icon: 'warning',
                    showCancelButton: true,
                    allowOutsideClick: false,
                    allowEscapeKey: false,
                    confirmButtonText: 'OK, go Home'
                });                
                if (!warnAnsw) return;

            }
            else {
                // Show 'Stop Robot' button
                const warnAnsw = await showSyncedPopup({
                    title: 'Stop Robot',
                    text: "The system will stop the robot now. Are you sure?",
                    icon: 'warning',
                    showCancelButton: true,
                    allowOutsideClick: false,
                    allowEscapeKey: false,
                    confirmButtonText: 'OK, Stop Robot'
                });
                if (!warnAnsw) return;
            }

            // Homing
            batteryMonitor.setShouldAutoRestart(false);
            robotStopClick();

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

        updateUI();

        secondBtn.disabled = false;
    }    
}