const WebSocket = require('ws');
const fs = require('fs');
const path = require('path');

// In-memory state storage
let pageState = {
    isPowered: false,
    isRunning: false,
    pipelineState: {}
};

// Percorso file per persistenza stato
const stateFile = path.join(__dirname, 'state.json');

// Carica lo stato salvato all'avvio del server
if (fs.existsSync(stateFile)) {
    pageState = JSON.parse(fs.readFileSync(stateFile, 'utf-8'));
}

// WebSocket setup
const wss = new WebSocket.Server({ noServer: true });

wss.on('connection', (ws) => {
    console.log("🔗 Nuovo client connesso");
    
    // Invia lo stato attuale solo se il WebSocket è pronto
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'stateUpdate', data: pageState }));
    }

    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            pageState = { ...pageState, ...data };
            broadcastState();
        } catch (error) {
            console.error("❌ Errore parsing WebSocket message:", error);
        }
    });

    ws.on('close', () => {
        console.log("❌ Client disconnesso");
    });
});

// Funzione per salvare lo stato su file
function saveState() {
    fs.writeFileSync(stateFile, JSON.stringify(pageState, null, 2));
}

// Funzione per inviare lo stato a tutti i client connessi
function broadcastState() {
    saveState(); // Salva lo stato prima di inviarlo ai client
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({ type: 'stateUpdate', data: pageState }));
        }
    });
}

// Ping periodico per mantenere attiva la connessione
setInterval(() => {
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.ping();
        }
    });
}, 30000); // Ping ogni 30 secondi

// Esportiamo WebSocket Server e funzione per gestire upgrade
module.exports = { wss, broadcastState, pageState };
