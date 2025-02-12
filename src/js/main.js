import { ROS_COMMANDS, LAUNCH_COMMANDS } from './constants.js';
import { saveButtonState, loadButtonState, updateUI , loadComponent} from './utils.js';
import { sendCommand, getRobotName, pingRemoteComputer, checkNodeStatus, checkStability, checkMotorsActivation, checkMovementController} from './api.js';
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
            const { value: initIMU } = await Swal.fire({
                title: 'Calibration in Progress',
                text: "Do not touch the robot during calibration.",
                icon: 'warning',
                showCancelButton: false,
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'OK, start calibration'
            });


            if (initIMU) {
                sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.IMU}`);
                
                let timerInterval;
                await Swal.fire({
                    title: 'Calibrating...',
                    html: 'Wait for 5 seconds',
                    timer: 5000,
                    timerProgressBar: true,
                    allowOutsideClick: false,
                    allowEscapeKey: false,
                    didOpen: () => {
                        Swal.showLoading()
                    }
                });

                // Check IMU connection
                const grepCommand = `grep 'Number of connected' ~/catkin_ws/src/AlterEGO_v2/alterego_robot/config/SystemCheck.txt`;
                const response = await fetch('/grep-command', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: grepCommand })
                });
                
                const data = await response.json();
                const numIMUs = parseInt(data.output.match(/\d+/)[0]);

                if (numIMUs !== 3) {
                    await Swal.fire('ERROR', 'IMU not Connected', 'error');
                    // Cleanup IMU
                    sendCommand(`source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill -a`);
                }
            }


            //Accendi il pilota
            sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.PILOT}`);


            //Liberare spazio per muoversi all'indietro
            const { value: initBackward } = await Swal.fire({
                title: 'Please ',
                text: "Clear the space behind the robot.",
                icon: 'warning',
                showCancelButton: false,
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'OK'
            });


            if (initBackward) {
                sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BACKWARD}`);
                
                const loadingAlert = Swal.fire({
                    title: 'Moving Backward...',
                    text: 'Please wait',
                    allowOutsideClick: false,
                    allowEscapeKey: false,
                    didOpen: () => {
                        Swal.showLoading();
                        
                        // Check node status every second
                        const checkInterval = setInterval(async () => {
                            const isNodeActive = await checkNodeStatus(`/${robotName}/wheels/backward`);
                            if (!isNodeActive) {
                                clearInterval(checkInterval);
                                Swal.close();
                            }
                        }, 1000);
                    }
                });

                await loadingAlert;
            }

            // Start the loading state
            Swal.fire({
                title: 'System Initialization',
                text: 'Raise the robot and wait...',
                allowOutsideClick: false,
                allowEscapeKey: false,
                didOpen: () => {
                    Swal.showLoading();
                }
            });

            const isStable = await checkStability(robotName);
            if (isStable) {
                // Avvia il controllo delle ruote
                sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.WHEELS}`);
                
                // Attendi che il sistema si stabilizzi
                await new Promise(resolve => setTimeout(resolve, 5000));
                Swal.close();
                
                // Attiva i motori delle braccia
                sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BODY_ACTIVATION}`);
                const motorsActivated = await checkMotorsActivation(robotName);                
                if (!motorsActivated) {
                    await Swal.fire('Error', 'Motors initialization failed', 'error');
                    return;
                }
                
                // Avvia il controllo del movimento
                sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BODY_MOVEMENT}`);
                const controllerStarted = await checkMovementController(robotName);                
                if (!controllerStarted) {
                    await Swal.fire('Error', 'Movement controller failed to start', 'error');
                    return;
                }
            } else {
                Swal.close();
                await Swal.fire('Error', 'Could not achieve stability', 'error');
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