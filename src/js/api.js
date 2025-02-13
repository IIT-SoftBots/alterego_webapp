import { ROS_COMMANDS, LAUNCH_COMMANDS } from './constants.js';

// Funzione per eseguire comandi generici
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

// Funzione per ottenere il nome del robot
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

// Funzione per verificare lo stato di un nodo ROS
export async function checkNodeStatus(nodeName) {
    try {
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

// Funzione per verificare la connessione con il computer remoto
export async function pingRemoteComputer() {
    try {
        const response = await fetch('/ping', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ ip: '192.168.0.70' })
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

export async function initializeIMU(robotName) {
    const { value: initIMU } = await Swal.fire({
        title: 'Calibration in Progress',
        text: "Do not touch the robot during calibration.",
        icon: 'warning',
        showCancelButton: false,
        allowOutsideClick: false,
        allowEscapeKey: false,
        confirmButtonText: 'OK, start calibration'
    })
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
        })
        // Check IMU connection
        const grepCommand = `grep 'Number of connected' ~/catkin_ws/src/AlterEGO_v2/alterego_robot/config/SystemCheck.txt`;
        const response = await fetch('/grep-command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: grepCommand })
        });
        
        const data = await response.json();
        const numIMUs = parseInt(data.output.match(/\d+/)[0])
        if (numIMUs !== 3) {
            await Swal.fire('ERROR', 'IMU not Connected', 'error');
            // Cleanup IMU
            sendCommand(`source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill -a`);
            return false;
        }
        return true;    
    }   
    return false;
}

export async function handleBackwardMovement(robotName) {
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
        return true;
    }
    return false;
}

export async function initializeSystem(robotName) {
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
            return false;
        }
        
        // Avvia il controllo del movimento
        sendCommand(`${ROS_COMMANDS.SETUP} && export ROBOT_NAME=${robotName} && ${LAUNCH_COMMANDS.BODY_MOVEMENT}`);
        const controllerStarted = await checkMovementController(robotName);                
        if (!controllerStarted) {
            await Swal.fire('Error', 'Movement controller failed to start', 'error');
            return false;
        }
        return true;
    } else {
        Swal.close();
        await Swal.fire('Error', 'Could not achieve stability', 'error');
        return false;
    }
}
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


export async function checkStability(robotName) {
    return new Promise((resolve) => {
        let checkInterval;
        let consecutiveNullReadings = 0;
        const MAX_NULL_READINGS = 5;
        const POLLING_INTERVAL = 500;
        
        let stableStartTime = null;
        let lastCheckTime = Date.now();
        
        checkInterval = setInterval(async () => {
            const now = Date.now();
            if (now - lastCheckTime < POLLING_INTERVAL) {
                return;
            }
            lastCheckTime = now;

            try {
                const value = await getTopicValue(`/${robotName}/imu/RPY`);
                console.log('IMU value:', value, 'Time:', new Date().toISOString());
                
                if (value === null) {
                    consecutiveNullReadings++;
                    console.warn(`Null reading #${consecutiveNullReadings}`);
                    
                    if (consecutiveNullReadings >= MAX_NULL_READINGS) {
                        console.error('Too many failed readings, stopping stability check');
                        clearInterval(checkInterval);
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
    });
}
export async function checkMotorsActivation(robotName) {
    let activationDetected = false;
    
    // Mostra l'alert di caricamento
    Swal.fire({
        title: 'Activating arms...',
        text: 'Waiting for activation',
        allowOutsideClick: false,
        allowEscapeKey: false,
        didOpen: () => {
            Swal.showLoading();
        }
    });

    // Funzione per controllare l'attivazione
    async function checkActivation() {
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
                    return true;
                }
            }
            return false;
        } catch (error) {
            console.error('Error checking arm movement:', error);
            return false;
        }
    }

    // Prova per 30 secondi (60 tentativi con 500ms di intervallo)
    for (let i = 0; i < 60 && !activationDetected; i++) {
        activationDetected = await checkActivation();
        if (!activationDetected) {
            await new Promise(resolve => setTimeout(resolve, 500));
        }
    }

    // Chiudi l'alert e ritorna il risultato
    Swal.close();
    return activationDetected;
}
export async function checkMovementController(robotName) {
    return new Promise((resolve) => {
        Swal.fire({
            title: 'Activating arms...',
            text: 'Please wait...',
            timer: 5000,
            timerProgressBar: true,
            allowOutsideClick: false,
            allowEscapeKey: false,
            didOpen: () => {
                Swal.showLoading();
            }
        }).then(() => {
            resolve(true);
        });
    });
}