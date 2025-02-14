import { ROS_COMMANDS, LAUNCH_COMMANDS } from './constants.js';
import { saveButtonState, loadButtonState, updateUI , loadComponent, updateBatteryGraphics} from './utils.js';
import { sendCommand, getRobotName, pingRemoteComputer, checkNodeStatus, checkStability, checkMotorsActivation, checkMovementController, initializeIMU, handleBackwardMovement, initializeSystem} from './api.js';
// Stato globale dell'applicazione
let state = {
    isPowered: false,
    isRunning: false
};
let socket;
// Inizializzazione dell'applicazione
async function initApp() {

    // Connessione al WebSocket
    socket = new WebSocket('ws://localhost:3000'); // Assicurati che l'URL corrisponda al tuo server
    socket.onopen = () => {
        console.log('WebSocket connected');
        // Invia lo stato corrente al server quando la connessione è aperta
        socket.send(JSON.stringify({ type: 'stateRequest' }));
    };

    socket.onmessage = (event) => {
        // Gestisci i messaggi ricevuti dal server
        const data = JSON.parse(event.data);

        if (data.type === 'stateUpdate') {
            // Aggiorna lo stato dell'applicazione con i dati ricevuti dal server
            state = { ...state, ...data.data };
            updateUI(state); // Aggiorna l'interfaccia utente
        }
    };

    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    socket.onclose = () => {
        console.log('WebSocket disconnected');
    };

    // Carica i components
    await loadComponent('button-grid', 'components/button-grid.html');
    await loadComponent('status-panel', 'components/status-panel.html');

    // Carica lo stato dal localStorage
    state.isPowered = loadButtonState('isPowered');
    state.isRunning = loadButtonState('isRunning');

    // Inizializza i controlli
    const powerBtn = document.getElementById('powerBtn');
    const startBtn = document.getElementById('startBtn');
    const configBtn = document.getElementById('configBtn');
    const homeBtn = document.getElementById('homeBtn');

    // Aggiungi event listeners
    powerBtn.addEventListener('click', handlePowerButtonClick);
    startBtn.addEventListener('click', handleStartButtonClick);
    configBtn.addEventListener('click', handleConfigButtonClick);
    homeBtn.addEventListener('click', handleHomeButtonClick);

    // Aggiorna l'UI con lo stato corrente
    updateUI(state);

    // Rimuovi classe loading
    document.querySelectorAll('.loading').forEach(el => {
        el.classList.add('loaded');
    });
}

// Handler per il pulsante power
async function handlePowerButtonClick() {
    try {

        console.log("Start")
        // Verifica connessione con il computer remoto
        const isRemoteComputerOnline = await pingRemoteComputer();
        if (!isRemoteComputerOnline) {
            Swal.fire('ERROR', 'Base computer not connected', 'error');
            return;
        }

        // Aggiorna stato
        state.isPowered = !state.isPowered;
        saveButtonState('isPowered', state.isPowered);

        if (state.isPowered) {
            // Avvia sequenza di accensione
            const robotName = await getRobotName();
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.ROSCORE}`);
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.USB_DETECTOR}`);

            // Initialize IMU
            const imuInitialized = await initializeIMU(robotName);
            if (!imuInitialized) {
                return;
            }

            // Accendi il pilota
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.PILOT}`);

            // Procede in retro quanto basta per uscire dalla stazione di ricarica wireless
            const backwardComplete = await handleBackwardMovement(robotName);
            if (!backwardComplete) {
                return;
            }

            // Initialize system (accende ruote, attiva braccia (activation e movement))
            const systemInitialized = await initializeSystem(robotName);
            if (!systemInitialized) {
                return;
            }

        } else {
            // Spegni tutto
            sendCommand(ROS_COMMANDS.CLEANUP);
            state.isRunning = false;
            saveButtonState('isRunning', false);
        }

        // Aggiorna UI
        updateUI(state);
    } catch (error) {
        console.error('Power button error:', error);
        state.isPowered = !state.isPowered; // Ripristina stato precedente
        updateUI(state);
        Swal.fire('Error', error.message, 'error');
    }
}

// Handler per il pulsante start
async function handleStartButtonClick() {
    try {
        state.isRunning = !state.isRunning;
        saveButtonState('isRunning', state.isRunning);
        updateUI(state);
    } catch (error) {
        console.error('Start button error:', error);
        state.isRunning = !state.isRunning;
        updateUI(state);
        Swal.fire('Error', error.message, 'error');
    }
}

// Handler per il pulsante config
function handleConfigButtonClick() {
    Swal.fire({
        title: 'Configuration',
        text: 'Configuration panel coming soon',
        icon: 'info'
    });
}

// Handler per il pulsante home
function handleHomeButtonClick() {
    Swal.fire({
        title: 'Home Position',
        text: 'Moving to home position...',
        icon: 'info',
        timer: 2000,
        timerProgressBar: true,
        showConfirmButton: false
    });
}

// Avvia l'applicazione quando il DOM è pronto
document.addEventListener('DOMContentLoaded', initApp);