const express = require('express');
const { exec } = require('child_process');

const app = express();
const port = 3000;
// Add this line to use express.json() middleware
app.use(express.json());

app.get('/', (req, res) => {
    res.send(`
        <html>
            <head>
                <title>GOLDEN EGO Connection</title>
                <style>
                    .button-container {
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                    }
                    .button {
                        height: 100px;
                        width: 400px;
                        margin-bottom: 50px;
                        font-size: 25px; /* Increase the font size */
                        background-color: grey; /* Set the initial background color */
                    }
                    .button.clicked {
                        background-color: green; /* Set the background color when clicked */
                    }
                </style>
            </head>
            <body>
                <h1>GOLDEN EGO Connection</h1>
                <div class="button-container">
                    <button class="button" onclick="sendCommand('source /opt/ros/noetic/setup.bash && roscore', 'killall -9 rosmaster')" id="button1">ROSCORE</button>
                    <button class="button" onclick="sendCommand('source .bashrc ./AlterEGO_v2/EGO_GUI/AV_com_Oculus.sh 192.168.0.62 video')" id="button2">Video</button>
                    <button class="button" onclick="sendCommand('source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROBOT_NAME=robot_goldenego && roslaunch alterego_robot imu.launch AlterEgoVersion:=2', 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill  /robot_goldenego/imu/Sensor  /robot_goldenego/imu/qb_interface_imu_node /robot_goldenego/alterego_state_publisher')" id="button3">IMU</button>
                    <button class="button" onclick="sendCommand('source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROBOT_NAME=robot_goldenego && roslaunch alterego_robot body_activation.launch AlterEgoVersion:=2', 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill /robot_goldenego/left/qb_manager /robot_goldenego/right/qb_manager')" id="button4">Body Activation</button>
                    <button class="button" onclick="sendCommand('source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROBOT_NAME=robot_goldenego && roslaunch alterego_robot body_movement.launch AlterEgoVersion:=2', 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill /robot_goldenego/right/arm_inv_dyn /robot_goldenego/left/arm_inv_dyn /robot_goldenego/head/head_inv_kin /robot_goldenego/left/arm_inv_kin_main /robot_goldenego/pitch_correction /robot_goldenego/right/arm_inv_kin_main')" id="button5">Body Movement</button>
                    <button class="button" onclick="sendCommand('source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROBOT_NAME=robot_goldenego && roslaunch alterego_robot pilot.launch AlterEgoVersion:=2', 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill /robot_goldenego/inbound_data /robot_goldenego/socket')" id="button6">Pilot Communication</button>
                </div>
                
                <script>
                    // Check if the button was previously clicked and update its state
                    function updateButtonState(buttonId) {
                        const button = document.getElementById(buttonId);
                        const clicked = localStorage.getItem(buttonId);
                        if (clicked === 'true') {
                            button.classList.add('clicked');
                        }
                    }
                    
                    // Save the button state when clicked
                    function saveButtonState(buttonId) {
                        const button = document.getElementById(buttonId);
                        const clicked = button.classList.contains('clicked');
                        localStorage.setItem(buttonId, clicked);
                    }
                    
                    // Reset the button state when clicked again
                    function resetButtonState(buttonId) {
                        const button = document.getElementById(buttonId);
                        button.classList.remove('clicked');
                        localStorage.removeItem(buttonId);
                    }
                    
                    // Send command and update button state
                    function sendCommand(command1, command2) {
                        const button = event.target;
                        if (button.classList.contains('clicked')) {
                            resetButtonState(button.id); // Reset the button state
                            sendFetchCommand(command2); // Send the second command
                        } else {
                            button.classList.add('clicked'); // Add the 'clicked' class to change the background color
                            saveButtonState(button.id); // Save the button state
                            sendFetchCommand(command1); // Send the first command
                        }
                    }
                    
                    // Function to send command using fetch
                    function sendFetchCommand(command) {
                        fetch('/send-command', {
                            method: 'POST',
                            headers: {
                                'Content-Type': 'application/json'
                            },
                            body: JSON.stringify({ command })
                        })
                        .then(response => response.json())
                        .then(data => {
                            console.log(data);
                            // Handle the response from the server if needed
                        })
                        .catch(error => {
                            console.error('Error:', error);
                            // Handle any errors that occurred during the request
                        });
                    }
                    
                    // Update button states on page load
                    window.addEventListener('load', () => {
                        updateButtonState('button1');
                        updateButtonState('button2');
                        updateButtonState('button3');
                        updateButtonState('button4');
                        updateButtonState('button5');
                        updateButtonState('button6');
                        updateButtonState('button7');
                    });
                </script>
            </body>
        </html>
    `);
});

app.post('/send-command', (req, res) => {
    const { command } = req.body;
    
    exec(`ssh goldenego-base@192.168.0.50 '${command}'`, (error, stdout, stderr) => {
        if (error) {
            console.error(`Error executing command: ${error.message}`);
            res.status(500).json({ error: 'Command execution failed' });
            return;
        }
        
        console.log(`Command output: ${stdout}`);
        res.json({ success: true });
    });
});

app.listen(port, () => {
    console.log(`Web app running at http://localhost:${port}`);
});