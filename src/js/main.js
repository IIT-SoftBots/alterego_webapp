// Importa le costanti e le funzioni utilities necessarie
import { handlePowerButtonClick } from './handlerButtonClick/handlePowerButtonClick.js';
import { handleStartButtonClick } from './handlerButtonClick/handleStartButtonClick.js';
import { handleConfigButtonClick } from './handlerButtonClick/handleConfigButtonClick.js';
import { handleHomeButtonClick } from './handlerButtonClick/handleHomeButtonClick.js';

// Importa le costanti e le funzioni utilities necessarie
import { updateUI, loadComponent } from './utils.js';
import { showSyncedPopup } from './api.js';

// Stato globale dell'applicazione
// Mantiene lo stato di accensione, esecuzione e UI
let state = {
    isPowered: false,    // Stato di accensione del sistema
    isRunning: false,    // Stato di esecuzione del sistema
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
    await loadComponent('button-grid', 'components/button-grid.html');
    await loadComponent('status-panel', 'components/status-panel.html');

    // Configura i pulsanti
    const powerBtn = document.getElementById('powerBtn');
    const startBtn = document.getElementById('startBtn');
    const configBtn = document.getElementById('configBtn');
    const homeBtn = document.getElementById('homeBtn');

    // Aggiungi event listener
    powerBtn.addEventListener('click', () => handlePowerButtonClick(ws, state));  
    startBtn.addEventListener('click', () => handleStartButtonClick(ws, state));  
    configBtn.addEventListener('click', () => handleConfigButtonClick(ws, state));
    homeBtn.addEventListener('click',  () => handleHomeButtonClick(ws, state));

    // Aggiorna UI iniziale
    updateUI(state);

    // Rimuovi classe loading
    document.querySelectorAll('.loading').forEach(el => {
        el.classList.add('loaded');
    });
}

// Avvia l'applicazione quando il DOM è pronto
document.addEventListener('DOMContentLoaded', initApp);