import { ROS_COMMANDS, LAUNCH_COMMANDS } from './constants.js';

/**
 * Executes a generic command through the server
 * @param {string} command - The command to execute
 * @returns {Promise<void>}
 */
export function sendCommand(command) {
    fetch('/execute', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ command }),
    })
    .then(() => console.log(`Command sent: ${command}`))
    .catch(error => console.error('Error sending command:', error));
}
/**
 * Retrieves the robot name from .bashrc
 * @returns {Promise<string>} The robot name
 * @throws {Error} If robot name cannot be retrieved
 */
export async function getRobotName() {
    try {
        const response = await fetch('/grep-command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 
                command: 'grep -oP "(?<=export ROBOT_NAME=).*" ~/.bashrc' 
            })
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        return data.output.trim();
    } catch (error) {
        console.error('Error getting robot name:', error);
        throw error;
    }
}

/**
 * Checks if a ROS node is currently running
 * @param {string} nodeName - Name of the node to check
 * @returns {Promise<boolean>} True if node is running, false otherwise
 * @throws {Error} If node status cannot be checked
 */
export async function checkNodeStatus(nodeName) {
    try {
        console.log("Checking the node in the list");

        const response = await fetch('/grep-command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 
                command: `${ROS_COMMANDS.SETUP} && rosnode list | grep ${nodeName}`
            })
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        return data.output.includes(nodeName);
    } catch (error) {
        console.error('Error checking node status:', error);
        throw error;
    }
}

/**
 * Checks connection with remote computer via ping
 * @returns {Promise<boolean>} True if connection successful, false otherwise
 */
export async function pingRemoteComputer() {
    try {
        const response = await fetch('/ping', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ ip: '192.168.0.110' })
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        return data.success;
    } catch (error) {
        console.error('Error pinging remote computer:', error);
        return false;
    }
}

/**
 * Initializes IMU sensors with calibration process
 * Shows synchronized popups for calibration steps
 * @param {WebSocket} ws - WebSocket connection for sync
 * @param {string} robotName - Name of the robot
 * @returns {Promise<boolean>} True if initialization successful
 */
export async function initializeIMU(ws, robotName) {
    try {
        // First popup - Calibration warning
        const initIMU = await showSyncedPopup(ws, {
            title: 'Calibration in Progress',
            text: "Do not touch the robot during calibration.",
            icon: 'warning',
            showCancelButton: false,
            allowOutsideClick: false,
            allowEscapeKey: false,
            confirmButtonText: 'OK, start calibration'
        });
        
        if (!initIMU) return false;

        // Send IMU command
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.IMU}`);
        console.log('IMU initialization command sent');

        // Add a small delay to ensure the first popup is fully closed
        await new Promise(resolve => setTimeout(resolve, 500));
        
        // Second popup - Calibration progress
        await showSyncedPopup(ws, {
            title: 'Calibrating...',
            html: 'Wait for 5 seconds',
            timer: 5000,
            timerProgressBar: true,
            allowOutsideClick: false,
            allowEscapeKey: false,
            showConfirmButton: false,
            willOpen: () => {
                if (Swal.getPopup()) {
                    Swal.showLoading();
                }
            }
        });


        // Check IMU connection
        const grepCommand = `grep 'Number of connected' ~/catkin_ws/src/AlterEGO_Adriano/alterego_robot/config/SystemCheck.txt`;
        const response = await fetch('/grep-command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: grepCommand })
        });
        
        const data = await response.json();
        const numIMUs = parseInt(data.output.match(/\d+/)?.[0] || '0');
        
        if (numIMUs !== 3) {
            await showSyncedPopup(ws, {
                title: 'ERROR',
                text: 'IMU not Connected',
                icon: 'error'
            });
            
            // Cleanup IMU
            sendCommand(`${ROS_COMMANDS.SETUP} && rosnode kill -a`);
            return false;
        }
        console.log('IMU Connected :', numIMUs);
        
        return true;
        
    } catch (error) {
        console.error('IMU initialization error:', error);
        await showSyncedPopup(ws, {
            title: 'Error',
            text: 'IMU initialization failed',
            icon: 'error'
        });
        return false;
    }
}
/**
 * Handles backward movement of the robot
 * Shows synchronized warning popup and monitors movement
 * @param {WebSocket} ws - WebSocket connection for sync
 * @param {string} robotName - Name of the robot
 * @returns {Promise<boolean>} True if movement completed
 */
export async function handleBackwardMovement(ws, robotName) {
    try {
        console.log("Starting backward movement...");
        await new Promise(resolve => setTimeout(resolve, 500));
        
        // First popup - Safety warning
        await showSyncedPopup(ws, {
            title: 'Safety Check',
            text: "Clear the space behind the robot.",
            icon: 'warning',
            showCancelButton: false,
            allowOutsideClick: false,
            allowEscapeKey: false,
            confirmButtonText: 'OK, start calibration'
        });
        

        // Send backward movement command
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BACKWARD}`);
        
        // Add a small delay before showing the progress popup
        await new Promise(resolve => setTimeout(resolve, 500));

        return new Promise((resolve) => {
            let checkInterval;
            
            // Show progress popup with node status checking
            Swal.fire({
                title: 'Moving Backward...',
                text: 'Please wait',
                allowOutsideClick: false,
                allowEscapeKey: false,
                showConfirmButton: false,
                didOpen: () => {
                    Swal.showLoading();
                    // Start checking node status
                    checkInterval = setInterval(async () => {
                        const isNodeActive = await checkNodeStatus(`/${robotName}/wheels/backward`);
                        if (!isNodeActive) {
                            clearInterval(checkInterval);
                            Swal.close();
                            // Show completion popup
                            await showSyncedPopup(ws, {
                                title: 'Complete',
                                text: 'Backward movement completed',
                                icon: 'success',
                                timer: 1500,
                                showConfirmButton: false
                            });
                            resolve(true);
                        }
                    }, 1000);
                },
                willClose: () => {
                    // Cleanup interval if popup is closed
                    if (checkInterval) {
                        clearInterval(checkInterval);
                    }
                }
            });
        });

    } catch (error) {
        console.error('Error in backward movement:', error);
        await showSyncedPopup(ws, {
            title: 'Error',
            text: 'Failed to move backward',
            icon: 'error'
        });
        return false;
    }
}
/**
 * Initializes the robot system
 * Checks stability, activates wheels and arms
 * @param {WebSocket} ws - WebSocket connection for sync
 * @param {string} robotName - Name of the robot
 * @returns {Promise<boolean>} True if initialization successful
 */
export async function initializeSystem(ws, robotName) {
    try {
        // Wait for system stabilization
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // Check stability
        const isStable = await checkStability(ws, robotName);
        if (!isStable) return false;

        // Start wheels control
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.WHEELS}`);
        await new Promise(r => setTimeout(r, 2000));
        
        // Activate arm motors
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BODY_ACTIVATION}`);
        
        console.log("Pre Checking arm motors activation...");

        // Check motors activation
        const motorsActivated = await checkMotorsActivation(ws, robotName);
        if (!motorsActivated) return false;
        
        await new Promise(r => setTimeout(r, 2000));
        console.log("Pre Checking arm motors movement...");
        
        // Start movement control
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BODY_MOVEMENT}`);
        await new Promise(r => setTimeout(r, 2000));
        
        const movementInitialized = await checkMovementController(ws);
        if (!movementInitialized) return false;
        
        return true;

    } catch (error) {
        console.error('Error in system initialization:', error);
        await showSyncedPopup(ws, {
            title: 'Error',
            text: 'System initialization failed',
            icon: 'error'
        });
        return false;
    }
}
/**
 * Retrieves value from a ROS topic with retry mechanism
 * @param {string} topic - ROS topic path
 * @returns {Promise<number|null>} Topic value or null if unavailable
 */
export async function getTopicValue(topic) {
    try {
        // Add retry mechanism
        let retries = 3;
        while (retries > 0) {
            try {
                const response = await fetch('/grep-command', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ 
                        command: `source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rostopic echo -n 1 ${topic}`
                    })
                });
                
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                
                const data = await response.json();
                if (!data || !data.output) {
                    console.warn('No data received from topic:', topic);
                    return null;
                }
                
                // Extract y value
                const match = data.output.match(/y:\s*([-]?\d*\.?\d*)/);
                if (!match) {
                    console.warn('Could not parse y value from:', data.output);
                    return null;
                }
                
                return parseFloat(match[1]);
            } catch (error) {
                retries--;
                if (retries === 0) throw error;
                console.warn(`Retry attempt left: ${retries}`);
                await new Promise(resolve => setTimeout(resolve, 1000)); // Wait 1s before retry
            }
        }
    } catch (error) {
        console.error('Error reading topic:', error);
        // If it's an SSH error, try to reconnect
        if (error.message.includes('SSH') || error.message.includes('Channel open failure')) {
            console.log('SSH connection lost, attempting to reconnect...');
            // Notify user of connection issue
            Swal.fire({
                title: 'Connection Lost',
                text: 'Attempting to reconnect...',
                icon: 'warning',
                showConfirmButton: false,
                allowOutsideClick: false,
                allowEscapeKey: false,
                timer: 2000
            });
        }
        return null;
    }
}

/**
 * Checks robot stability using IMU readings
 * @param {string} robotName - Name of the robot
 * @returns {Promise<boolean>} True if robot is stable
 */
export async function checkStability(ws, robotName) {
    try {
        console.log("Starting stability check...");
        await new Promise(resolve => setTimeout(resolve, 500));

        return new Promise((resolve) => {
            let checkInterval;
            let consecutiveNullReadings = 0;
            const MAX_NULL_READINGS = 5;
            const POLLING_INTERVAL = 500;
            
            let stableStartTime = null;
            
            // Show progress popup with stability checking
            showSyncedPopup(ws, {
                title: 'Checking Stability...',
                text: 'Please wait and raise the robot',
                allowOutsideClick: false,
                allowEscapeKey: false,
                showConfirmButton: false,
                didOpen: () => {
                    if (Swal.getPopup()) {
                        Swal.showLoading();
                    }
                    // Start checking stability
                    checkInterval = setInterval(async () => {
                        try {
                            const value = await getTopicValue(`/${robotName}/imu/RPY`);
                            console.log('IMU value:', value, 'Time:', new Date().toISOString());
                            
                            if (value === null) {
                                consecutiveNullReadings++;
                                console.warn(`Null reading #${consecutiveNullReadings}`);
                                
                                if (consecutiveNullReadings >= MAX_NULL_READINGS) {
                                    console.error('Too many failed readings');
                                    clearInterval(checkInterval);
                                    Swal.close();
                                    await showSyncedPopup(ws, {
                                        title: 'Error',
                                        text: 'Stability check failed - No readings',
                                        icon: 'error'
                                    });
                                    resolve(false);
                                    return;
                                }
                                return;
                            }
                            
                            consecutiveNullReadings = 0;
                            
                            if (value >= -0.2 && value <= 0.1) {
                                if (!stableStartTime) {
                                    stableStartTime = Date.now();
                                    console.log('Started stability timer');
                                }
                                
                                const stableTime = Date.now() - stableStartTime;
                                console.log('Stable for:', (stableTime/1000).toFixed(1), 'seconds');
                                
                                if (stableTime >= 2000) {
                                    console.log('Stability achieved');
                                    clearInterval(checkInterval);
                                    Swal.close();
                                    await showSyncedPopup(ws, {
                                        title: 'Complete',
                                        text: 'Stability check successful',
                                        icon: 'success',
                                        timer: 1500,
                                        showConfirmButton: false
                                    });
                                    resolve(true);
                                    return;
                                }
                            } else {
                                if (stableStartTime) {
                                    console.log('Stability lost, resetting timer');
                                    stableStartTime = null;
                                }
                            }
                        } catch (error) {
                            console.error('Error in stability check:', error);
                        }
                    }, POLLING_INTERVAL);
                },
                willClose: () => {
                    if (checkInterval) {
                        clearInterval(checkInterval);
                    }
                }
            });
        });

    } catch (error) {
        console.error('Error in stability check:', error);
        await showSyncedPopup(ws, {
            title: 'Error',
            text: 'Stability check failed',
            icon: 'error'
        });
        return false;
    }
}
/**
 * Checks if arm motors are properly activated
 * @param {WebSocket} ws - WebSocket connection for sync
 * @param {string} robotName - Name of the robot
 * @returns {Promise<boolean>} True if motors are activated
 */
export async function checkMotorsActivation(ws, robotName) {
    return new Promise((resolve) => {
        let activationDetected = false;
        let checkCount = 0;
        const MAX_ATTEMPTS = 60; // 30 seconds (60 * 500ms)
        console.log("Checking arm motors activation...");
        // Show progress popup with activation checking
        showSyncedPopup(ws, {
            title: 'Activating arms...',
            text: 'Waiting for activation',
            allowOutsideClick: false,
            allowEscapeKey: false,
            showConfirmButton: false,
            willOpen: () => {
                if (Swal.getPopup()) {
                    Swal.showLoading();
                }

                // Start checking activation in intervals
                const checkInterval = setInterval(async () => {
                    try {
                        const response = await fetch('/grep-command', {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/json' },
                            body: JSON.stringify({ 
                                command: `source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rostopic echo -n 1 /${robotName}/alterego_state/upperbody | grep left_meas_arm_shaft`
                            })
                        });
                        
                        const data = await response.json();
                        if (data?.output) {
                            const values = data.output.match(/[-]?\d*\.?\d+/g);
                            if (values?.some(v => Math.abs(parseFloat(v)) > 0.01)) {
                                console.log('Activation detected:', values);
                                clearInterval(checkInterval);
                                Swal.close();
                                resolve(true);
                                return;
                            }
                        }

                        checkCount++;
                        if (checkCount >= MAX_ATTEMPTS) {
                            clearInterval(checkInterval);
                            Swal.close();
                            resolve(false);
                        }
                    } catch (error) {
                        console.error('Error checking arm movement:', error);
                        checkCount++;
                    }
                }, 500);

                // Cleanup on popup close
                Swal.getPopup().addEventListener('swal-closed', () => {
                    clearInterval(checkInterval);
                    resolve(false);
                });
            }
        });
    });
}
/**
 * Checks if movement controller is running
 * @param {WebSocket} ws - WebSocket connection for sync
 * @returns {Promise<boolean>} True if controller is active
 */
export async function checkMovementController(ws) {
    return new Promise((resolve) => {
        showSyncedPopup(ws, {
            title: 'Activating movement...',
            text: 'Please wait...',
            timer: 5000,
            timerProgressBar: true,
            allowOutsideClick: false,
            allowEscapeKey: false,
            showConfirmButton: false,
            willOpen: () => {
                if (Swal.getPopup()) {
                    Swal.showLoading();
                }
            }
        }).then(() => {
            resolve(true);
        });
    });
}
/**
 * Initializes WebSocket connection for state synchronization
 * @param {Object} state - Current application state
 * @param {Function} updateUI - Function to update UI with new state
 * @returns {WebSocket} Configured WebSocket instance
 */
export function initializeWebSocket(state, updateUI) {
    const socket = new WebSocket('ws://localhost:3000');
    
    socket.onopen = () => {
        console.log('WebSocket connected');
        socket.send(JSON.stringify({ type: 'stateRequest' }));
    };

    socket.onmessage = (event) => {
        const data = JSON.parse(event.data);

        if (data.type === 'stateUpdate') {
            Object.assign(state, data.data);
            updateUI(state);
        }
    };

    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    socket.onclose = () => {
        console.log('WebSocket disconnected');
    };

    return socket;
}
/**
 * Shows a synchronized popup across all connected clients
 * @param {WebSocket} ws - WebSocket connection
 * @param {Object} popupData - SweetAlert2 configuration object
 * @param {string} popupData.title - Popup title
 * @param {string} popupData.text - Popup message
 * @param {string} [popupData.icon] - Popup icon type
 * @param {boolean} [popupData.showCancelButton] - Show cancel button
 * @param {boolean} [popupData.allowOutsideClick] - Allow clicking outside
 * @param {boolean} [popupData.allowEscapeKey] - Allow ESC key to close
 * @returns {Promise<boolean>} True if confirmed, false otherwise
 */
export function showSyncedPopup(ws, popupData) {
    // Evita duplicati
    if (Swal.isVisible()) return Promise.resolve(false);
    
    // Notifica altri client
    ws.send(JSON.stringify({
        type: 'popup',
        data: popupData
    }));

    // Visualizza il popup localmente e ritorna la Promise
    return Swal.fire({
        ...popupData,
        // Handler per la chiusura del popup
        didClose: () => {
            // Notifica la chiusura agli altri client
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'closePopup'
                }));
            }
        }
    }).then((result) => {
        // Gestione click sul pulsante OK
        if (result.isConfirmed) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'closePopup'
                }));
            }
        }
        return result.isConfirmed;
    });
}