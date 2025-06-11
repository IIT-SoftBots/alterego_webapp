// Importa le costanti e le funzioni utilities necessarie
import { handleMainButtonClick } from './handlerButtonClick/handleMainButtonClick.js';
import { ClickMonitor, clickMonitorClose, UnlockClickMonitor } from './handlerButtonClick/handleAdminMenuButtonClick.js';
import { handleSecondButtonClick } from './handlerButtonClick/handleSecondButtonClick.js';

// Importa le costanti e le funzioni utilities necessarie
import { updateUI, loadComponent, closeAdminMenu, settingsAction, setVolume, closeVolumeMenu } from './utils.js';
import { pingRemoteComputer, sendCommand, showSyncedPopup, startBatteryCheck } from './api.js';
import { batteryMonitor } from './batterymonitor.js';
import { LAUNCH_COMMANDS, ROS_COMMANDS, STATE , initializeConfig } from './constants.js';
import { overrideInitRobotState, restartAuto } from './workflow.js';

// Stato globale dell'applicazione
// Mantiene lo stato di accensione, esecuzione e UI
export let state = {
    isPowered: false,    // Stato di accensione del sistema
    isRunning: false,    // Stato di esecuzione del sistema
    pipelineState: 0,    // Workflow State
    uiState: {
        activePopup: null,     // Popup attualmente visualizzato
        notifications: []      // Coda delle notifiche (max 10)
    }
};

// Connessione WebSocket globale
let ws = new WebSocket(`ws://localhost:3000`);
export { ws };

/**
 * Inizializza la connessione WebSocket
 * Gestisce gli aggiornamenti di stato e la riconnessione automatica
 */
function initWebSocket() {
    
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
            updateUI();
            
            // Gestione sincronizzata dei popup
            if (!state.uiState.activePopup && Swal.isVisible()) {
                Swal.close();  // Chiudi popup se non più attivo
            }
            else if (state.uiState.activePopup) {
                showSyncedPopup(state.uiState.activePopup);
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

    // Initialize config first
    await initializeConfig();
    console.log('Config initialized');
   
    // Carica i componenti UI
    const resLoad = await loadComponent('button-grid', 'components/button-grid.html');
    const resComp = await loadComponent('status-panel', 'components/status-panel.html');

    // Inizializza WebSocket
    initWebSocket();

    // Configura i pulsanti
    const mainBtn = document.getElementById('mainBtn');
    const secondBtn = document.getElementById('secondBtn');
    const settingsBtn = document.getElementById('settingsBtn');
    const volumeBtn = document.getElementById('volumeBtn');
    const closeBtn = document.getElementById('closeBtn');
    const logoBtn = document.getElementById('alterEgoLogo');
    const unlockOverlay = document.getElementById('unlockOverlay');
    
    // Configura monitor per clicks e batteria
    const monitor = new ClickMonitor(logoBtn);
    const unlockMonitor = new UnlockClickMonitor(unlockOverlay);

    // Aggiungi event listener
    mainBtn.addEventListener('click', () => handleMainButtonClick());   
    secondBtn.addEventListener('click',  () => handleSecondButtonClick());
    settingsBtn.addEventListener('click',  () => settingsAction());
    volumeBtn.addEventListener('click',  () => setVolume());
    closeBtn.addEventListener('click',  () => clickMonitorClose(monitor, unlockMonitor));

    // Gestione del click fuori dal popup per chiuderlo
    document.getElementById('popupOverlay').addEventListener('click', function(e) {
        if (e.target === this) {
            closeAdminMenu();
            closeVolumeMenu();
        }
    });

    // Rimuovi classe loading
    document.querySelectorAll('.loading').forEach(el => {
        el.classList.add('loaded');
    });
    
    if (state.pipelineState == STATE.RESTART_AUTO){
        restartAuto();

        state.pipelineState = STATE.INIT;
            
        ws.send(JSON.stringify({
            type: 'stateUpdate',
            data: { pipelineState: state.pipelineState }
        }));    
    }
    else {
        overrideInitRobotState();
    }

    // Aggiorna l'interfaccia utente 
    updateUI();

    // Start Battery Monitor
    const isRemoteComputerOnline = await pingRemoteComputer();
    if (isRemoteComputerOnline) {        
        // Start battery monitor script
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.BATTERY_MONITOR.START}`);
        await new Promise(r => setTimeout(r, 1000));
        console.log("Start Battery Monitoring");
    }
    batteryMonitor.start(state.pipelineState);
    
    // Start checking battery
    startBatteryCheck();
}

// Avvia l'applicazione quando il DOM è pronto, MA SOLO se non è la pagina custom tasks
document.addEventListener('DOMContentLoaded', () => {
    if (!window.isCustomTasksPage) {
        initApp();
    } 
});
