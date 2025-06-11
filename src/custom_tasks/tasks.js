import { STATE, initializeConfig } from '../js/constants.js';
import { state, ws } from '../js/main.js';
import { customTaskDefinitions } from './customTasks.js';

let lastPipelineState = STATE.RESTART_AUTO;

// Function to render custom task buttons
function renderCustomTaskButtons() {
    const container = document.getElementById('customTasksContainer');
    if (!container) {
        console.error('customTasksContainer not found');
        return;
    }
    container.innerHTML = ''; // Clear existing buttons

    customTaskDefinitions.forEach(taskDef => {
        const button = document.createElement('button');
        button.id = `task-btn-${taskDef.id}`;
        button.innerHTML = `<i class="fas fa-list-check"></i><span>${taskDef.buttonLabel}</span>`;
        button.classList.add('task-button', 'control-button');

        button.addEventListener('click', async () => {
            button.disabled = true;
            try {
                const confirmResult = await showLocalPopup({
                    title: taskDef.popupTitle,
                    text: taskDef.popupText,
                    icon: 'warning',
                    showCancelButton: true,
                    allowOutsideClick: false,
                    allowEscapeKey: false,
                    confirmButtonText: 'OK, Execute Task'
                });

                if (!confirmResult) {
                    return; // User cancelled the task
                }

                // Execute the task action defined in the taskDef
                const taskResult = await taskDef.action();

                if (taskResult && taskResult.message) {
                    // Show a popup with the result message
                    await showLocalPopup({
                        title: taskResult.success ? 'Task Success' : 'Task Error',
                        text: taskResult.message,
                        icon: taskResult.success ? 'success' : 'error',
                    });
                    console.log(`Task result "${taskDef.buttonLabel}":`, taskResult);
                }

            } catch (error) {
                console.error(`Error while executing task "${taskDef.buttonLabel}":`, error);
                await showLocalPopup({
                    title: 'Error in Task Execution',
                    text: error.message || 'An unexpected error occurred while executing the task.',
                    icon: 'error'
                });
            } finally {
                // Re-enable the button after task execution
                button.disabled = false;
            }
        });
        container.appendChild(button);
    });

    // Apply colors to buttons after they are created
    applyButtonColors();

    // After rendering buttons, update their state based on the current pipeline state
    updateTaskButtonsState();
}

function applyButtonColors() {
    const buttons = document.querySelectorAll('#customTasksContainer .task-button');
    const count = buttons.length;
    buttons.forEach((btn, i) => {
        const hue = Math.round((360 / count) * i);
        btn.style.backgroundColor = `hsl(${hue}, 70%, 55%)`;
        btn.style.color = '#fff';
        btn.style.border = 'none';
    });
}

// Websocket Handlers
ws.onopen = async () => {
    console.log('WebSocket connection established for custom tasks page');
    ws.send(JSON.stringify({ type: 'requestInitialState'}));
    
    await initializeConfig();
    renderCustomTaskButtons();
};

ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === 'stateUpdate') {
        const newStateData = message.data;

        if (newStateData && typeof newStateData === 'object'){
            Object.assign(state, newStateData);
        }

        // Update Tasks User Interface
        updateTaskButtonsState();
    }
};

ws.onclose = async () => {
    // Handle automatic reconnection
    await showLocalPopup({
        title: 'WebApp Closed',
        text: 'The WebApp has been closed. This page will be closed as well in a few time. Please refresh the page to reconnect.',
        icon: 'warning',
        allowOutsideClick: false,
        allowEscapeKey: false,
        timer: 8000,
        timerProgressBar: true,
        showConfirmButton: false
    });
    // Reload the web page and force clear the cache
    window.location.reload(true);
};


function updateTaskButtonsState() {
    console.log("Updated state for task buttons: ", state.pipelineState);

    // Logic to enable/disable all task buttons based on pipelineState
    const disableTasksCondition = state.pipelineState !== STATE.WORK_MODE && state.pipelineState !== STATE.PAUSED;

    const taskButtons = document.querySelectorAll('#customTasksContainer .task-button');
    taskButtons.forEach(button => {
        button.disabled = disableTasksCondition;
    });

    // Popup logic based on pipelineState
    if (state.pipelineState === STATE.WORK_MODE){
        if (Swal.isVisible()){
            Swal.close();
        }
    }
    else if (state.pipelineState !== lastPipelineState) {
        var popupText = '';
        switch(state.pipelineState){
            case STATE.ACTIVATE_ROBOT: case STATE.STAND_UP:
                popupText = 'Robot is starting. Please wait...';
                break;
            case STATE.PAUSED:
                popupText = 'Robot Paused';
                break;
            default:
                popupText = 'Waiting for the User to start the Robot';
                break;
        }
        // Avoid to show the popup if it is already visible
        if (!Swal.isVisible() || !Swal.getConfirmButton()?.hasAttribute('listenerAttached')) { // Heuristica per popup di task
             showLocalPopup({
                title: 'Warning',
                text: popupText,
                icon: 'warning',
                allowOutsideClick: false,
                allowEscapeKey: false,
                showConfirmButton: false,                
            });
        }
    }

    lastPipelineState = state.pipelineState;
}

function showLocalPopup(popupData) {
    // Avoid duplicate popups
    if (Swal.isVisible()){
        const currentSwalHasCancel = Swal.getCancelButton() && Swal.getCancelButton().style.display !== 'none';
        const newSwalHasCancel = popupData.showCancelButton;

        if (!currentSwalHasCancel || !newSwalHasCancel) {
            Swal.close();
        } else if (currentSwalHasCancel && !newSwalHasCancel) {
            return Promise.resolve(false); // Emulate no confirmation
        }
    }

    return Swal.fire({
        ...popupData,
    }).then((result) => {
        if (popupData.showCancelButton) {
            const confirmButton = Swal.getConfirmButton();
            if (confirmButton) confirmButton.setAttribute('listenerAttached', 'true');
        }
        return result.isConfirmed;
    });
}

// Assures the DOM is ready before rendering buttons
// if 'ws.onopen' is not sufficiently delayed.
// Alternatively, you can call `initializeConfig` and `renderCustomTaskButtons`
// within a DOMContentLoaded event if `ws` connects very early.
// document.addEventListener('DOMContentLoaded', () => {
//     if (ws.readyState === WebSocket.OPEN) { // If ws is already open
//         (async () => {
//             await initializeConfig();
//             renderCustomTaskButtons();
//         })();
//     }
// });