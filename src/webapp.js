const express = require('express');
const { exec } = require('child_process');
const path = require('path');
const bodyParser = require('body-parser');
const { Client } = require('ssh2');
const fs = require('fs');


// Importa WebSocket
const { wss } = require('./js/websocket');

const app = express();
const port = 3000;

// Middleware setup
app.use(bodyParser.json());
app.use(express.json());

// Static file serving
app.use(express.static(__dirname));
app.use('/images', express.static(path.join(__dirname, 'images')));

// Start server
const server = app.listen(port, '0.0.0.0', () => {
    console.log(`Server running at http://0.0.0.0:${port}`);
});

// Collegamento WebSocket al server HTTP
server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
    });
});

// SSH setup
let sshConnected = false;
const sshClient = new Client();

function connectSSH() {
    if (sshConnected) return Promise.resolve();

    return new Promise((resolve, reject) => {
        const connectionParams = {
            host: '192.168.0.110',
            username: 'alterego-base',
            privateKey: fs.readFileSync('/home/alterego-vision/.ssh/id_rsa')
        };

        sshClient.connect(connectionParams);

        sshClient.on('ready', () => {
            console.log('Connected via SSH!');
            sshConnected = true;
            resolve();
        });

        sshClient.on('error', (err) => {
            console.error('Error connecting via SSH:', err);
            sshConnected = false;
            reject(err);
        });
    });
}

function execute(command) {
    return new Promise((resolve, reject) => {
        sshClient.exec(command, (err, stream) => {
            if (err) {
                reject(err);
                return;
            }

            let output = '';
            let errorOutput = '';

            stream.on('data', (data) => {
                output += data.toString();
            });

            stream.stderr.on('data', (data) => {
                errorOutput += data.toString();
            });

            stream.on('close', () => {
                if (errorOutput) {
                    console.error('Command error:', errorOutput);
                }
                resolve(output);
            });
        });
    });
}

// Routes
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.post('/execute', async (req, res) => {
    try {
        if (!sshConnected) {
            await connectSSH();
        }
        // Esegui il comando senza attendere l'output
        sshClient.exec(req.body.command, (err) => {
            if (err) {
                console.error('Execute error:', err);
                res.status(500).json({ success: false });
                return;
            }
            // Rispondi subito con successo
            res.json({ success: true });
        });
    } catch (error) {
        console.error('Execute error:', error);
        res.status(500).json({ success: false });
    }
});

app.post('/grep-command', async (req, res) => {
    try {
        if (!sshConnected) {
            await connectSSH();
        }
        const output = await execute(req.body.command);
        res.json({ output });
    } catch (error) {
        console.error('Grep command error:', error);
        res.status(500).json({ error: error.message });
    }
});

app.post('/send-videocommand', (req, res) => {
    exec(req.body.command, (error, stdout, stderr) => {
        if (error) {
            console.error('Video command error:', error);
            res.status(500).json({ success: false, error: error.message });
            return;
        }
        res.json({ success: true, output: stdout });
    });
});

app.post('/ping', (req, res) => {
    const ip = req.body.ip;
    exec(`ping -c 1 ${ip}`, (error) => {
        res.json({ success: !error });
    });
});


// Error handling
app.use((err, req, res, next) => {
    console.error('Application error:', err);
    res.status(500).send('Something broke!');
});

process.on('uncaughtException', (err) => {
    console.error('Uncaught Exception:', err);
    process.exit(1);
});

process.on('unhandledRejection', (reason) => {
    console.error('Unhandled Rejection:', reason);
    process.exit(1);
});

// Start server
// app.listen(port, '0.0.0.0', () => {
//     console.log(`Server running at http://0.0.0.0:${port}`);
// });