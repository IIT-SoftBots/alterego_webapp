import { pauseProcedures } from "../js/workflow.js";

/**
 * This is the file where you, the user, can define custom tasks.
 * Each task is an object in the `customTaskDefinitions` array.
 * 
 * Each task object should have the following properties:
 *  - id: A unique string identifier for the task (e.g., 'myCustomScan').
 *  - buttonLabel: The text that will appear on the button for this task.
 *  - popupTitle: The title of the confirmation popup (e.g., 'Confirm Scan').
 *  - popupText: The main message in the confirmation popup.
 *  - action: An asynchronous (`async`) function that contains the steps to execute for this task.
 *            This function will be called after the user confirms the popup.
 *            You can use `await` for asynchronous operations.
 *            If needed, you can import and use functions like `sendCommand` from `../js/api.js`
 *            or other utility functions. For example:
 *            // import { sendCommand, showSyncedPopup } from '../js/api.js';
 *            // import { ROS_COMMANDS, LAUNCH_COMMANDS } from '../js/constants.js';
 */
export const customTaskDefinitions = [
    {
        id: 'userTask1',
        buttonLabel: 'Pause the Robot',
        popupTitle: 'PAUSE ROBOT',
        popupText: 'Robot will be paused. Continue?',
        action: async () => {
            console.log('Custom Task: Starting...');
            // Sample logic:
            try {
                pauseProcedures();

                // Emulate a duration for the task
                await new Promise(resolve => setTimeout(resolve, 5000));

                return { success: true, message: 'Custom Task accomplished.' };
            } catch (error) {
                return { success: false, message: `Error: ${error.message}` };
            }
        }
    },
    {
        id: 'userTask2',
        buttonLabel: 'Execute Task 2',
        popupTitle: 'Task 2',
        popupText: 'Task routines will be executed. Continue?',
        action: async () => {
            
            // Placeholder as test:
            alert('Action "Task 2" executed');
            console.log('Task finished.');
            return { success: true, message: 'Task accomplished.' };
        }
    },
    // Add here more custom tasks following the same structure
];

// Note:
// Functions 'action' should be 'async' if they perform asynchronous operations
// (e.g., API calls like `sendCommand` or if they use `await showSyncedPopup`).
// The main file `tasks.js` (or a similar file that manages the task page UI)
// should import `customTaskDefinitions` and use it to dynamically create buttons
// and associate the actions defined here.