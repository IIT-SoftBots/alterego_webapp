// Import necessary constants and utility functions
import { handleMainButtonClick } from './handlerButtonClick/handleMainButtonClick.js';
import { ClickMonitor, clickMonitorClose, UnlockClickMonitor } from './handlerButtonClick/handleAdminMenuButtonClick.js';
import { handleSecondButtonClick } from './handlerButtonClick/handleSecondButtonClick.js';

// Import necessary constants and utility functions
import { updateUI, loadComponent, closeAdminMenu, settingsAction, setVolume, closeVolumeMenu, loadAndApplySettings } from './utils.js';
import { pingRemoteComputer, sendCommand, showSyncedPopup, startBatteryCheck } from './api.js';
import { batteryMonitor } from './batterymonitor.js';
import { LAUNCH_COMMANDS, NUC_VISION_IP, ROS_COMMANDS, STATE , initializeConfig, retrieveRobotConfig } from './constants.js';
import { overrideInitRobotState, restartAuto } from './workflow.js';

// Global application state
// Maintains power, execution, and UI state
export let state = {
    isPowered: false,    // System power state
    isRunning: false,    // System execution state
    pipelineState: 0,    // Workflow State
    uiState: {
        activePopup: null,     // Currently displayed popup
        notifications: []      // Notification queue (max 10)
    }
};

// Global WebSocket connection
export let ws;

/**
 * Initializes the WebSocket connection.
 */
function initWebSocket() {
    return new Promise((resolve, reject) => {
        ws = new WebSocket(`ws://${NUC_VISION_IP}:3000`);
        
        ws.onopen = () => {
            console.log('WebSocket connected');
            ws.send(JSON.stringify({ type: 'requestInitialState' }));
            resolve(ws);
        };

        ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            reject(error); 
        };
    });
}

/**
 * Handles state updates and automatic reconnection.
 */
function handleWebSocket() {

    ws.onmessage = (event) => {
        const message = JSON.parse(event.data);
        if (message.type === 'stateUpdate') {
            const newState = message.data;
            // state = { ...state, ...newState }; 
            Object.assign(state, newState);
            updateUI();
            if (!state.uiState.activePopup && Swal.isVisible()) {
                Swal.close();
            } else if (state.uiState.activePopup) {
                showSyncedPopup(state.uiState.activePopup);
            }
        }
    };

    ws.onclose = () => {
        console.log('WebSocket disconnected. Attempting to reconnect in 5 seconds...');
        setTimeout(() => {
            initWebSocket().catch(err => console.error("Reconnection attempt failed:", err));
        }, 5000);
    };
}

async function initWebPage() {

    // Initialize config first
    await initializeConfig();
    console.log('Config initialized');

    // Load settings on startup
    await loadAndApplySettings();
   
    // Initialize WebSocket
    try {
        await initWebSocket(); // Wait for WebSocket to be established
        console.log('WebSocket connection established and ready.');
    } catch (error) {
        console.error('Failed to establish WebSocket connection:', error);
        showSyncedPopup({ title: 'Connection Error', text: 'Could not connect to the server. Please try refreshing the page.', icon: 'error', showConfirmButton: true });
        return;
    }

    const isRemoteComputerOnline = await pingRemoteComputer();
    console.log('Remote computer online:', isRemoteComputerOnline);
    if (isRemoteComputerOnline) {        
        await retrieveRobotConfig();
    }
    else {
        console.error('Remote computer is not online. Cannot retrieve robot configuration.');
        Swal.fire({
            title: 'Connection Error',
            text: "Could not connect to base NUC. Please check it is online. The page will be reloaded in a few seconds...",
            icon: 'error',
            allowOutsideClick: false,
            allowEscapeKey: false,
            timer: 8000,
            timerProgressBar: true,
            showConfirmButton: false
        }).then(() => {
            // Reload the web page and force clear the cache
            window.location.reload(true);
            
        });

        return;
    }
}

/**
 * Initializes the application.
 * Loads components and sets up event listeners.
 */
async function initApp() {

    // Load UI components
    const resLoad = await loadComponent('button-grid', 'components/button-grid.html');
    const resComp = await loadComponent('status-panel', 'components/status-panel.html');

    handleWebSocket(); // Set up WebSocket message handling
    
    // Configure buttons
    const mainBtn = document.getElementById('mainBtn');
    const secondBtn = document.getElementById('secondBtn');
    const settingsBtn = document.getElementById('settingsBtn');
    const volumeBtn = document.getElementById('volumeBtn');
    const closeBtn = document.getElementById('closeBtn');
    const logoBtn = document.getElementById('alterEgoLogo');
    const unlockOverlay = document.getElementById('unlockOverlay');
    
    // Configure click and battery monitors
    const monitor = new ClickMonitor(logoBtn);
    const unlockMonitor = new UnlockClickMonitor(unlockOverlay);

    // Add event listeners
    mainBtn.addEventListener('click', () => handleMainButtonClick());   
    secondBtn.addEventListener('click',  () => handleSecondButtonClick());
    settingsBtn.addEventListener('click',  () => settingsAction());
    volumeBtn.addEventListener('click',  () => setVolume());
    closeBtn.addEventListener('click',  () => clickMonitorClose(monitor, unlockMonitor));

    // Handle click outside the popup to close it
    document.getElementById('popupOverlay').addEventListener('click', function(e) {
        if (e.target === this) {
            closeAdminMenu();
            closeVolumeMenu();
        }
    });

    // Remove loading class
    document.querySelectorAll('.loading').forEach(el => {
        el.classList.add('loaded');
    });
    
    if (state.pipelineState == STATE.RESTART_AUTO){ // Ensure STATE is defined
        restartAuto();

        state.pipelineState = STATE.INIT; // Ensure STATE.INIT is defined
            
        if (ws && ws.readyState === WebSocket.OPEN) { // Check if ws is ready before sending
            ws.send(JSON.stringify({
                type: 'stateUpdate',
                data: { pipelineState: state.pipelineState }
            }));
        }  
    }
    else {
        overrideInitRobotState();
    }

    // Update the user interface
    updateUI();

    // Start Battery Monitor
    const isRemoteComputerOnline = await pingRemoteComputer();
    if (isRemoteComputerOnline) {     
        
        // Start battery monitor script
        sendCommand(`${ROS_COMMANDS.SETUP} && ${LAUNCH_COMMANDS.BATTERY_MONITOR.START}`);
        await new Promise(r => setTimeout(r, 1000));
        console.log("Start Battery Monitoring");
    }
    batteryMonitor.start(state.pipelineState); // Ensure batteryMonitor is defined
    
    // Start checking battery
    startBatteryCheck();
}

// Start the application when the DOM is ready
document.addEventListener('DOMContentLoaded', async () => {
    
    // Initialize config and load settings on startup
    await initWebPage(); // Wait for WebSocket to be established
    
    if (!window.isCustomTasksPage) {
        // Do additional initialization for the main application
        initApp();
    } 
});
