// Importa le costanti e le funzioni utilities necessarie
import { handleMainButtonClick } from './handlerButtonClick/handleMainButtonClick.js';
import { ClickMonitor } from './handlerButtonClick/handleAdminMenuButtonClick.js';
import { handleSecondButtonClick } from './handlerButtonClick/handleSecondButtonClick.js';

// Importa le costanti e le funzioni utilities necessarie
import { updateUI, loadComponent, closeAdminMenu, settingsAction } from './utils.js';
import { showSyncedPopup } from './api.js';
import { batteryMonitor } from './batterymonitor.js';

// Stato globale dell'applicazione
// Mantiene lo stato di accensione, esecuzione e UI
let state = {
    isPowered: false,    // Stato di accensione del sistema
    isRunning: false,    // Stato di esecuzione del sistema
    pipelineState: 0,    // Workflow State
    uiState: {
        activePopup: null,     // Popup attualmente visualizzato
        notifications: []      // Coda delle notifiche (max 10)
    }
};

// Connessione WebSocket globale
let ws;

/**
 * Inizializza la connessione WebSocket
 * Gestisce gli aggiornamenti di stato e la riconnessione automatica
 */
function initWebSocket() {
    ws = new WebSocket(`ws://localhost:3000`);
    
    // Handler per i messaggi in arrivo
    ws.onmessage = (event) => {
        const message = JSON.parse(event.data);
        // Aggiungi handler per la connessione
        ws.onopen = () => {
            console.log('WebSocket connected');
            // Richiedi lo stato iniziale appena connesso
            ws.send(JSON.stringify({ type: 'requestInitialState' }));
        };
        if (message.type === 'stateUpdate') {
            // Aggiorna lo stato locale con i nuovi dati
            const newState = message.data;
            state = { ...state, ...newState };
            
            // Aggiorna l'interfaccia utente
            updateUI(state);
            
            // Gestione sincronizzata dei popup
            if (!state.uiState.activePopup && Swal.isVisible()) {
                Swal.close();  // Chiudi popup se non più attivo
            }
            else if (state.uiState.activePopup) {
                showSyncedPopup(ws, state.uiState.activePopup);
            }
        }
    };
    
    // Gestione riconnessione automatica
    ws.onclose = () => {
        setTimeout(initWebSocket, 5000);  // Riprova ogni 5 secondi
    };
}


/**
 * Inizializza l'applicazione
 * Carica i componenti e configura gli event listener
 */
async function initApp() {
    // Inizializza WebSocket
    initWebSocket();
    
    // Carica i componenti UI
    const resLoad = await loadComponent('button-grid', 'components/button-grid.html');
    const resComp = await loadComponent('status-panel', 'components/status-panel.html');

    // Configura i pulsanti
    const mainBtn = document.getElementById('mainBtn');
    const secondBtn = document.getElementById('secondBtn');
    const settingsBtn = document.getElementById('settingsBtn');
    const closeBtn = document.getElementById('closeBtn');
    const logoBtn = document.getElementById('alterEgoLogo');

    // Configura monitor per clicks e batteria
    const monitor = new ClickMonitor(ws, logoBtn);

    // Aggiungi event listener
    mainBtn.addEventListener('click', () => handleMainButtonClick(ws, state));   
    secondBtn.addEventListener('click',  () => handleSecondButtonClick(ws, state));
    settingsBtn.addEventListener('click',  () => settingsAction());
    closeBtn.addEventListener('click',  () => monitor.executeCloseFunction());

    // Gestione del click fuori dal popup per chiuderlo
    document.getElementById('popupOverlay').addEventListener('click', function(e) {
        if (e.target === this) {
            closeAdminMenu();
        }
    });

    // Rimuovi classe loading
    document.querySelectorAll('.loading').forEach(el => {
        el.classList.add('loaded');
    });

    // Aggiorna l'interfaccia utente
    updateUI(state);
        
    // Start Battery Monitor
    batteryMonitor.start(ws, state.pipelineState);
}

// Avvia l'applicazione quando il DOM è pronto
document.addEventListener('DOMContentLoaded', initApp);
