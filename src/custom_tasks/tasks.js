import { STATE } from '../js/constants.js';
import { state, ws } from '../js/main.js';
import { customTaskDefinitions } from './customTasks.js';

let lastPipelineState = STATE.RESTART_AUTO;
const debugTaskPage = false; // Set to true to enable debug mode for the custom tasks page

// Websocket Handlers
async function handleTasksWebSocket() {
    
    console.log('WebSocket connection established for custom tasks page');
    ws.send(JSON.stringify({ type: 'requestInitialState'}));
  
    renderCustomTaskButtons();
    
    // Redefine the WebSocket message handler for this page
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
}
   

// Function to render custom task buttons
function renderCustomTaskButtons() {
    const container = document.getElementById('customTasksContainer');
    if (!container) {
        console.error('customTasksContainer not found');
        return;
    }
    container.innerHTML = ''; // Clear existing buttons

    const numberOfTasks = customTaskDefinitions.length;

    // --- Styling for the container to grow and distribute height ---
    container.style.display = 'grid';
    container.style.flexGrow = '1';
    container.style.width = '100%';
    container.style.maxWidth = '100%'; // Max width for two columns
    
    if (numberOfTasks > 0 && numberOfTasks < 4) { // Single column
        container.style.gridTemplateColumns = '1fr';
        // Distribute available height equally among the rows
        container.style.gridTemplateRows = `repeat(${numberOfTasks}, 1fr)`;
    } else if (numberOfTasks >= 4) { // Two columns
        const numRows = Math.ceil(numberOfTasks / 2);
        container.style.gridTemplateColumns = 'repeat(2, 1fr)';
        // Distribute available height equally among the rows
        container.style.gridTemplateRows = `repeat(${numRows}, 1fr)`;
    } else { // No tasks
        container.style.flexGrow = '0'; // Don't grow if no tasks
        container.style.display = 'none'; // Hide if no tasks
    }
    // --- End of container styling ---

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
                    button.disabled = false; // Re-enable button if user cancelled
                    return; 
                }

                // Execute the task action defined in the taskDef
                const taskResult = await taskDef.action();

                if (taskResult && taskResult.message) {
                    console.log(`Task "${taskDef.buttonLabel}" executed`);
                    console.log('Result: ', taskResult.success ? 'Task Success' : 'Task Error');
                    console.log('Message: ', taskResult.message);
                }

            } catch (error) {
                console.error(`Error while executing task "${taskDef.buttonLabel}":`, error);
                await showLocalPopup({
                    title: 'Error in Task Execution',
                    text: error.message || 'An unexpected error occurred while executing the task.',
                    icon: 'error'
                });
            } finally {
                button.disabled = false;
            }
        });
        container.appendChild(button);
    });

    // Apply colors to buttons after they are created
    applyButtonColors();
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

function updateTaskButtonsState() {
    console.log("Updated state for task buttons: ", state.pipelineState);

    // Logic to enable/disable all task buttons based on pipelineState
    const disableTasksCondition = state.pipelineState !== STATE.WORK_MODE;

    const taskButtons = document.querySelectorAll('#customTasksContainer .task-button');
    
    // Debug mode allows all buttons to be enabled regardless of pipelineState
    if (!debugTaskPage) {
        taskButtons.forEach(button => {
            button.disabled = disableTasksCondition;
        });
    }

    // Popup logic based on pipelineState
    if (!disableTasksCondition){
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
            case STATE.STOPPING:
                popupText = 'Robot Stopping';
                break;
            default:
                popupText = 'Waiting for the User to start the Robot';
                break;
        }
        
        // Avoid to show the popup if it is already visible
        if (!debugTaskPage && (!Swal.isVisible() || !Swal.getConfirmButton()?.hasAttribute('listenerAttached'))) {
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

// Start the application when the DOM is ready
document.addEventListener('DOMContentLoaded', async () => {
     
    while (typeof ws === 'undefined' || ws.readyState !== WebSocket.OPEN) {
        await new Promise(resolve => setTimeout(resolve, 100)); // Wait for ws to be defined and connected
    }

    // Wait until ws is defined and connected before calling handleTasksWebSocket
    await handleTasksWebSocket();
});