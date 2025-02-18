import { showSyncedPopup } from '../api.js';

/**
 * Gestisce il click sul pulsante di configurazione
 * Mostra il pannello di configurazione (placeholder)
 */
export async function handleConfigButtonClick(ws, state) {
    showSyncedPopup(ws, {
        title: 'Configuration',
        text: 'Configuration panel coming soon',
        icon: 'info',
        showConfirmButton: true,
        confirmButtonText: 'OK'
    });
}
