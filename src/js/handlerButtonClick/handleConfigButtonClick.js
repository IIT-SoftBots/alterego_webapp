import { showSyncedPopup } from '../api.js';

/**
 * Gestisce il click sul pulsante di configurazione
 * Mostra il pannello di configurazione (placeholder)
 */
export async function handleConfigButtonClick(ws, state) {
    showSyncedPopup(ws, {
        title: 'Configuration',
        text: 'Do you want to close the app?',
        icon: 'info',
        showConfirmButton: true,
        confirmButtonText: 'OK',
        preConfirm: () => {
            ws.send(JSON.stringify({ type: 'closeApp' })); // Invia un messaggio di chiusura al server
        }
    });
}