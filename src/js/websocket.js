// websocket.js
const WebSocket = require('ws');
const fs = require('fs');
const path = require('path');
const { exec } = require('child_process');

// In-memory state storage with add of UI state
let pageState = {
    isPowered: false,
    isRunning: false,
    pipelineState: 0,
    uiState: {
        activePopup: null,
        notifications: []
    }
};

const stateFile = path.join(__dirname, 'state.json');
console.log("📁 State saved in:", stateFile);
if (fs.existsSync(stateFile)) {
    try {
        pageState = JSON.parse(fs.readFileSync(stateFile, 'utf-8'));
    }
    catch (error) {
        console.error("❌ Error parsing state file:", error);
    }
}

const wss = new WebSocket.Server({ noServer: true });

wss.on('connection', (ws) => {
    console.log("🔗 New client connected");
    
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ 
            type: 'stateUpdate', 
            data: pageState 
        }));
    }

    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            
            switch(data.type) {
                case 'requestInitialState':
                    // Immediately send the current state to the client
                    ws.send(JSON.stringify({ 
                        type: 'stateUpdate', 
                        data: pageState 
                    }));
                    break;
                    
                case 'stateUpdate':
                    if (data.data) {
                        pageState = { 
                            ...pageState, 
                            ...data.data,
                            uiState: {
                                ...pageState.uiState,
                                ...(data.data.uiState || {})
                            }
                        };
                    }
                    break;
                    
                case 'popup':
                    if (data.data) {
                        pageState = {
                            ...pageState,
                            uiState: {
                                ...pageState.uiState,
                                activePopup: data.data
                            }
                        };
                    }
                    break;
                    
                case 'notification':
                    if (data.data) {
                        const notifications = [...pageState.uiState.notifications, data.data].slice(-10);
                        pageState = {
                            ...pageState,
                            uiState: {
                                ...pageState.uiState,
                                notifications
                            }
                        };
                    }
                    break;
                    
                case 'closePopup':
                    pageState = {
                        ...pageState,
                        uiState: {
                            ...pageState.uiState,
                            activePopup: null
                        }
                    };
                    broadcastState();  // Assure broadcast after closing popup
                    break;
                case 'closeApp':
                    // Executes the command to close Firefox
                    exec('pkill -f firefox', (error, stdout, stderr) => {
                        if (error) {
                            console.error(`Error closing Firefox: ${error}`);
                        } else {
                            console.log('Firefox closed successfully');
                        }
                
                        // Executes the command to close the process on port 3000
                        exec('lsof -t -i :3000 | xargs kill -9', (error, stdout, stderr) => {
                            if (error) {
                                console.error(`Error closing process on port 3000: ${error}`);
                                return;
                            }
                            console.log('Process on port 3000 closed successfully');
                        });
                    });
                    break;
            }
            
            broadcastState();
        } catch (error) {
            console.error("❌ Error parsing WebSocket message:", error);
        }
    });

    ws.on('close', () => {
        console.log("❌ Client disconnected");
    });
});

function saveState() {
    fs.writeFileSync(stateFile, JSON.stringify(pageState, null, 2));
}

function broadcastState() {
    saveState();
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({ 
                type: 'stateUpdate', 
                data: pageState 
            }));
        }
    });
}

setInterval(() => {
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.ping();
        }
    });
}, 30000);

module.exports = { wss, broadcastState, pageState };