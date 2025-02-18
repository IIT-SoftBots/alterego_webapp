import { showSyncedPopup } from '../api.js';
/**
 * Gestisce il click sul pulsante Home
 * Avvia il movimento verso la posizione home
 */
export async function handleHomeButtonClick(ws, state) {
    showSyncedPopup(ws, {
        title: 'Home Position',
        text: 'Moving to home position...',
        icon: 'info',
        timer: 2000,
        timerProgressBar: true,
        showConfirmButton: false
    });
}