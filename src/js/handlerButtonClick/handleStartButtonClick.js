import { updateUI } from '../utils.js';

/**
 * Gestisce il click sul pulsante Start
 * Avvia il movimento verso la posizione home
 */
export async function handleStartButtonClick(ws, state) {
    try {
        state.isRunning = !state.isRunning;
        updateUI(state);
    } catch (error) {
        console.error('Start button error:', error);
        state.isRunning = !state.isRunning;
        updateUI(state);
        Swal.fire('Error', error.message, 'error');
    }
}