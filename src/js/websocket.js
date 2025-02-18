// websocket.js
const WebSocket = require('ws');
const fs = require('fs');
const path = require('path');

// In-memory state storage con aggiunta di UI state
let pageState = {
    isPowered: false,
    isRunning: false,
    pipelineState: {},
    uiState: {
        activePopup: null,
        notifications: []
    }
};

const stateFile = path.join(__dirname, 'state.json');

if (fs.existsSync(stateFile)) {
    pageState = JSON.parse(fs.readFileSync(stateFile, 'utf-8'));
}

const wss = new WebSocket.Server({ noServer: true });

wss.on('connection', (ws) => {
    console.log("🔗 Nuovo client connesso");
    
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
                    // Invia immediatamente lo stato corrente al nuovo client
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
                    broadcastState();  // Assicurati che lo stato venga propagato
                    break;
            }
            
            broadcastState();
        } catch (error) {
            console.error("❌ Errore parsing WebSocket message:", error);
        }
    });

    ws.on('close', () => {
        console.log("❌ Client disconnesso");
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