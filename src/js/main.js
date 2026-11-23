// Importa le costanti e le funzioni utilities necessarie
import { handleMainButtonClick } from './handlerButtonClick/handleMainButtonClick.js';
import { ClickMonitor, clickMonitorClose, UnlockClickMonitor } from './handlerButtonClick/handleAdminMenuButtonClick.js';
import { handleSecondButtonClick } from './handlerButtonClick/handleSecondButtonClick.js';

// Importa le costanti e le funzioni utilities necessarie
import { updateUI, loadComponent, closeAdminMenu, settingsAction, setVolume, closeVolumeMenu } from './utils.js';
import { getRobotName, pingRemoteComputer, sendCommand, showSyncedPopup, startBatteryCheck } from './api.js';
import { batteryMonitor } from './batterymonitor.js';
import { LAUNCH_COMMANDS, ROS_COMMANDS, STATE , initializeConfig } from './constants.js';
import { overrideInitRobotState, restartAuto } from './workflow.js';
import { initTroubleshoot } from './troubleshoot.js';   // TEST COPY

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

// Nome del robot nel file .bashrc
let robotName;

// TEST COPY: resolved when the server sends the saved state (first stateUpdate)
let gotInitialState;
const initialStateReady = new Promise(resolve => { gotInitialState = resolve; });

/**
 * Inizializza la connessione WebSocket
 * Gestisce gli aggiornamenti di stato e la riconnessione automatica
 */
function initWebSocket() {
    ws = new WebSocket(`ws://${location.host}`);   // TEST COPY: was ws://localhost:3000
   
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
            gotInitialState();   // TEST COPY
            
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
 * TEST COPY: on page load keep the robot as it is.
 * - waits for the state saved by the server (max 3 s)
 * - if ROS is off but the saved state says the robot is on, the saved state is old: go back to Init
 * No command is sent to the robot.
 */
async function restoreStateWithoutCleanup() {
    await Promise.race([initialStateReady, new Promise(r => setTimeout(r, 3000))]);
    let masterUp = null;
    try {
        const ctrl = new AbortController();
        const t = setTimeout(() => ctrl.abort(), 5000);   // base off or booting: don't keep the page hidden
        const res = await fetch('/api/check', { signal: ctrl.signal });
        clearTimeout(t);
        const data = await res.json();
        if (data.ok) masterUp = data.masterUp;
    } catch (e) { console.warn('Robot check not available', e); }

    if (masterUp === false && (state.pipelineState !== STATE.INIT || state.isPowered)) {
        state.pipelineState = STATE.INIT;
        state.isPowered = false;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'stateUpdate', data: { pipelineState: state.pipelineState, isPowered: state.isPowered } }));
        }
    }
    console.log('Page loaded without cleanup. ROS running:', masterUp, 'state:', state.pipelineState);
}

/**
 * Inizializza l'applicazione
 * Carica i componenti e configura gli event listener
 */
async function initApp() {
    // Initialize config first
    await initializeConfig();
    console.log('Config initialized');
    // Inizializza WebSocket
    initWebSocket();

    // Carica i componenti UI
    const resLoad = await loadComponent('button-grid', 'components/button-grid.html');
    const resComp = await loadComponent('status-panel', 'components/status-panel.html');
    initTroubleshoot();   // TEST COPY: pulsante "Controlla il robot"

    // Configura i pulsanti
    const mainBtn = document.getElementById('mainBtn');
    const secondBtn = document.getElementById('secondBtn');
    const settingsBtn = document.getElementById('settingsBtn');
    const volumeBtn = document.getElementById('volumeBtn');
    const closeBtn = document.getElementById('closeBtn');
    const logoBtn = document.getElementById('alterEgoLogo');
    const unlockOverlay = document.getElementById('unlockOverlay');
    
    // Configura monitor per clicks e batteria
    const monitor = new ClickMonitor(ws, logoBtn);
    const unlockMonitor = new UnlockClickMonitor(ws, unlockOverlay);

    // Get Robot Name
    robotName = "robot_adriano"; //await getRobotName();

    // Aggiungi event listener
    mainBtn.addEventListener('click', () => handleMainButtonClick(ws, state, robotName));   
    secondBtn.addEventListener('click',  () => handleSecondButtonClick(ws, state, robotName));
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
        restartAuto(ws, state, robotName);

        state.pipelineState = STATE.INIT;
            
        ws.send(JSON.stringify({
            type: 'stateUpdate',
            data: { pipelineState: state.pipelineState }
        }));    
    }
    else {
        // TEST COPY: the original called overrideInitRobotState(), which sends
        // "rosnode kill -a && killall -9 rosmaster" on EVERY page load (it switched the robot off).
        // Here the page only reads the saved state and does not send any command.
        await restoreStateWithoutCleanup();
    }

    // Aggiorna l'interfaccia utente 
    updateUI(state);

    // Start Battery Monitor
    var isRemoteComputerOnline = await pingRemoteComputer();
    while (!isRemoteComputerOnline) {
        // Mostra errore se non connesso
        const popupData = {
            title: 'Errore',
            text: 'Il computer del robot (base) non risponde ancora. Riprovo tra 5 secondi…',
            icon: 'warning',
            timer: 5000, // Removed timer
            timerProgressBar: true, // Removed timer progress bar
            showConfirmButton: false,
        };
        
        // Try once again
        isRemoteComputerOnline = await pingRemoteComputer();
    }

    if (isRemoteComputerOnline) {        
        // Start battery monitor script
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BATTERY_MONITOR.START}`);
        await new Promise(r => setTimeout(r, 1000));
        console.log("Start Battery Monitoring");
    }
    batteryMonitor.start(ws, state.pipelineState);
    
    // Start checking battery
    startBatteryCheck(robotName);
}

// Avvia l'applicazione quando il DOM è pronto
document.addEventListener('DOMContentLoaded', initApp);
