const express = require('express');
const { exec } = require('child_process');
const path = require('path');
const bodyParser = require('body-parser');
const { Client } = require('ssh2');
const fs = require('fs');
const yaml = require('js-yaml');
const os = require('os'); // Add os module to get system information
const proxy = require('express-http-proxy'); // Importa express-http-proxy

// Importa WebSocket
const { wss } = require('./js/websocket.js');
const { CONF_FEATURES } = require('./js/constants.js');

const app = express();
const port = 3000;
// Initialize with a default value
let NUC_BASE_IP = '192.168.88.110';
let NUC_VISION_IP = '192.168.88.111';

// Get current username
const currentUsername = os.userInfo().username;
// Create the remote username once for consistent use
const remoteUsername = currentUsername.replace('-vision', '-base');
console.log(`\nCurrent Username: ${currentUsername} \nRemote Username: ${remoteUsername}\n`);

// Middleware setup
app.use(bodyParser.json());
app.use(express.json());

// Static file serving
app.use(express.static(__dirname));
app.use('/images', express.static(path.join(__dirname, 'images')));

// Configure timeouts
app.keepAliveTimeout = 35000; // 35 seconds
app.headersTimeout = 40000; // 40 seconds

// Get host IP and derive NUC_BASE_IP using hostname -I
// Fixed function with proper Promise handling
function getMyIP() {
    return new Promise((resolve, reject) => {
        exec('hostname -I', async (error, stdout, stderr) => {
            if (error) {
                console.error('Error getting IPs:', error);
                reject(error);
                return;
            }
            
            // Get first IP address from hostname -I output
            const hostIP = stdout.trim().split(' ')[0];
            if (hostIP === '127.0.0.1') {
                console.log(`Only localhost found (127.0.0.1), try again`);
                resolve(null);          
            }
            resolve({ hostIP });
        });
    });
}

function getNucIPs() {
    return new Promise(async (resolve, reject) => {
        
        let ip = await getMyIP();        
        while (ip == null){
            await new Promise(r => setTimeout(r, 2000)); 
            ip = await getMyIP();
        }

        // Only remove the last digit from the last octet
        const ipParts = ip.hostIP.split('.');
        const lastOctet = ipParts[3];
        ipParts[3] = lastOctet.slice(0, -1) + '0';
        
        const nucIP = ipParts.join('.');
        
        console.log(`\nCurrent UserIP: ${ip.hostIP} \nRemote UserIP: ${nucIP}\n`);
        resolve({ base: nucIP, vision: ip.hostIP });
        
    });
}
// Use the async function properly
getNucIPs().then(async ips => {
    NUC_BASE_IP = ips.base;
    NUC_VISION_IP = ips.vision;
    
    // Get the robot version from .bashrc
    let robotVersion = 2; // Default value
    try {
        // Execute the command directly without SSH
        const result = await new Promise((resolve, reject) => {
            exec('grep -oP "(?<=export AlterEgoVersion=).*" ~/.bashrc', (error, stdout, stderr) => {
                if (error) {
                    console.warn('Could not find AlterEgoVersion in .bashrc, using default value');
                    resolve(null);
                } else {
                    resolve(stdout.trim());
                }
            });
        });
        
        if (result) {
            const version = parseInt(result, 10);
            if (!isNaN(version)) {
                robotVersion = version;
                console.log(`Robot Version: ${robotVersion}\n`);
            }
        }
    } catch (error) {
        console.error('Error getting robot version:', error);
    }

    // Get the robot name from .bashrc
    let robotName = 'robot_alterego'; // Default value
    try {
        // Execute the command directly without SSH
        const result = await new Promise((resolve, reject) => {
            exec('grep -oP "(?<=export ROBOT_NAME=).*" ~/.bashrc', (error, stdout, stderr) => {
                if (error) {
                    console.warn('Could not find Robot Name in .bashrc, using default value');
                    resolve(null);
                } else {
                    resolve(stdout.trim());
                }
            });
        });
        
        if (result) {
            robotName = result;
            console.log(`Robot Name: ${robotName}\n`);            
        }
    } catch (error) {
        console.error('Error getting robot name:', error);
    }
    
    // Update config endpoint to include robot version
    app.get('/api/config', (req, res) => {
        res.json({ 
            NUC_BASE_IP: NUC_BASE_IP, 
            NUC_VISION_IP: NUC_VISION_IP,
            AlterEgoVersion: robotVersion,
            RobotName: robotName,  
        });
    });
    
    // Start server only after IPs are set
    startServer();
}).catch(err => {
    console.error('Failed to get IPs:', err);
    // Use default IPs
    startServer();
});

// Move server startup to a function
function startServer() {   
    const server = app.listen(port, '0.0.0.0', () => {
        //console.log(`Server running at http://0.0.0.0:${port}`);
    });
    
    // Collegamento WebSocket al server HTTP
    server.on('upgrade', (request, socket, head) => {
        wss.handleUpgrade(request, socket, head, (wsInstance) => {
            wss.emit('connection', wsInstance, request);
        });
    });

      // Report the app access URLs
    console.log(`Access main app via http://${NUC_VISION_IP}:${port}`);

    // Initialize custom tasks server
    initializeTasksServer();
}

// SSH setup
let sshConnected = false;
const sshClient = new Client();

function connectSSH() {
    if (sshConnected) return Promise.resolve();

    return new Promise((resolve, reject) => {
        const connectionParams = {
            host: NUC_BASE_IP,
            username: remoteUsername,
            privateKey: fs.readFileSync(`/home/${currentUsername}/.ssh/id_rsa`)
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
        // sshClient.on('error', (err) => {
        //     console.error('SSH error:', err);
        //     sshConnected = false;
        //     // Prova a riconnetterti
        //     setTimeout(() => {
        //     if (!sshConnected) connectSSH();
        //     }, 5000);
        // });

    });
}

function executeLocal(command) {
    return new Promise((resolve, reject) => {
        exec(`bash -c '${command}'`, (error, stdout, stderr) => {
            if (error) {
                reject(error);
                return;
            }
            resolve(stdout);
        });
    });
}

app.post('/execute-local', async (req, res) => {
    try {
        const output = await executeLocal(req.body.command);
        res.json({ success: true, output });
    } catch (error) {
        console.error('Execute local error:', error);
        res.status(500).json({ success: false, error: error.message });
    }
});

app.post('/grep-command-local', async (req, res) => {
    try {
        const output = await executeLocal(req.body.command);
        res.json({ output });
    } catch (error) {
        console.error('Grep command local error:', error);
        res.status(500).json({ error: error.message });
    }
});

function execute(command) {
    return new Promise((resolve, reject) => {
        sshClient.exec(`bash -lc "${command}"`, (err, stream) => {
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
    const client = new Client(); // Create a new SSH client for this request
    const connectionParams = {
        host: NUC_BASE_IP,
        username: remoteUsername,
        privateKey: fs.readFileSync(`/home/${currentUsername}/.ssh/id_rsa`)
    };
    let output = '';
    let errorOutput = '';

    try {
        client.on('ready', () => {
            client.exec(req.body.command, (err, stream) => {
                if (err) {
                    console.error('Execute error:', err);
                    res.status(500).json({ success: false, error: err.message });
                    client.end(); // Close the connection
                    return;
                }

                stream.on('data', (data) => {
                    output += data.toString();
                });

                stream.stderr.on('data', (data) => {
                    errorOutput += data.toString();
                });

                /*stream.on('close', () => {
                    client.end(); // Close the connection
                    if (errorOutput) {
                        console.error('Command error:', errorOutput);
                        res.status(500).json({ success: false, error: errorOutput });
                    } else {
                        res.json({ success: true, output });
                    }
                });*/

                // Rispondi subito con successo
                res.json({ success: true });
            });
        });

        client.on('error', (clientErr) => { // For SSH connection-level errors
            console.error(`SSH client connection error for command "${req.body.command}": ${clientErr.message}`);
            if (!res.headersSent) {
                // Attempt to inform the HTTP client only if no response has been sent yet
                if (clientErr.code === 'ECONNRESET' && req.body.command && req.body.command.includes('shutdown')) {
                    console.log('ECONNRESET during shutdown command, considered expected if command was initiated.');
                    res.json({ success: true, message: "Shutdown command likely initiated; connection reset is expected." });
                } else {
                    res.status(500).json({ success: false, error: `SSH client error: ${clientErr.message}` });
                }
            } else {
                // Headers already sent, just log the error.
                console.warn(`SSH client.on('error') ("${clientErr.message}") occurred, but headers already sent. Cannot send HTTP error response.`);
            }
            // Always ensure the client is closed on connection-level errors.
            if (client && !client.destroyed) client.end();
        });

        client.connect(connectionParams); // Connect the client
    } catch (error) {
        console.error('Execute error:', error);
        res.status(500).json({ success: false, error: error.message });
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


// -------------- Configuration Features ------------------
const CONFIGURATION_FILE_PATH = path.join(__dirname, 'config', 'robot_webapp_configuration.yaml');
const DEFAULT_FEATURES = {};
for (const key in CONF_FEATURES) {
    if (Object.hasOwnProperty.call(CONF_FEATURES, key) && CONF_FEATURES[key] && typeof CONF_FEATURES[key].value !== 'undefined') {
        DEFAULT_FEATURES[key] = CONF_FEATURES[key].value;
    } else {
        // Fallback in case the structure is not as expected or value is undefined
        console.warn(`Key "${key}" in CONF_FEATURES does not have a property 'value' defined. Set to false as default.`);
        DEFAULT_FEATURES[key] = false;
    }
}

function readFeatures() {    
    if (fs.existsSync(CONFIGURATION_FILE_PATH)) {
        try {
            const fileContents = fs.readFileSync(CONFIGURATION_FILE_PATH, 'utf8');
            const loadedFeatures = yaml.load(fileContents);
            // Assures all default keys are present
            // and loaded values overwrite defaults only if present in the file
            const mergedFeatures = { ...DEFAULT_FEATURES };
            for (const key in loadedFeatures) {
                if (Object.hasOwnProperty.call(loadedFeatures, key) && Object.hasOwnProperty.call(mergedFeatures, key)) {
                    mergedFeatures[key] = loadedFeatures[key];
                }
            }
            return mergedFeatures;
        } catch (error) {
            console.error('Error reading or parsing features file, using defaults:', error);
            return { ...DEFAULT_FEATURES };
        }
    }
    return { ...DEFAULT_FEATURES};
}

function writeFeatures(features) {
    try {
        const dir = path.dirname(CONFIGURATION_FILE_PATH);
        if (!fs.existsSync(dir)) {
            fs.mkdirSync(dir, { recursive: true });
        }
        const yamlStr = yaml.dump(features);
        fs.writeFileSync(CONFIGURATION_FILE_PATH, yamlStr, 'utf8');
        console.log('Features saved to:', CONFIGURATION_FILE_PATH);
    } catch (error) {
        console.error('Error writing features file:', error);
    }
}

// API route to get features
app.get('/api/features', (req, res) => {
    const features = readFeatures();
    res.json(features);
});

// API route to update features
app.post('/api/features', (req, res) => {
    const newFeatures = req.body;
    // Basic validation: ensure all default keys are present if partial update is not desired
    const updatedFeatures = { ...readFeatures(), ...newFeatures };
    writeFeatures(updatedFeatures);
    res.json({ success: true, message: 'Features updated successfully.', features: updatedFeatures });
});


// -------------- Custom Task Server ------------------
async function initializeTasksServer() {
    const appTasks = express();
    const portTasks = 3001;

    // Middleware and static as the main app
    appTasks.use(bodyParser.json());
    appTasks.use(express.json());

    // Serves tasks-index.html page as root
    appTasks.get('/', (req, res) => {
        res.sendFile(path.join(__dirname, 'custom_tasks', 'tasks-index.html'));
    });

    // --- API MIRROR: all the main app routes are replicated on appTasks ---
    const routesToMirror = [
        { method: 'get', path: '/api/config' },
        { method: 'get', path: '/api/features' },
        { method: 'post', path: '/execute-local' },
        { method: 'post', path: '/grep-command-local' },
        { method: 'post', path: '/execute' },
        { method: 'post', path: '/grep-command' },
        { method: 'post', path: '/send-videocommand' },
        { method: 'post', path: '/ping' }
        // add other routes here if needed
    ];

    routesToMirror.forEach(route => {
        const proxyTargetHost = `0.0.0.0:${port}`;
        const proxyTarget = `http://${proxyTargetHost}`;
        
        // Utilizzo di express-http-proxy
        appTasks[route.method](route.path, proxy(proxyTarget, {
            proxyReqPathResolver: function (req) {
                const newPath = req.originalUrl;
                //console.log(`[API Mirror - appTasks:${portTasks}] Proxying ${req.method} ${req.originalUrl} (req.path: ${req.path}) to ${proxyTarget}${newPath}`);
                return newPath;
            },
            userResDecorator: function(proxyRes, proxyResData, userReq, userRes) {
                //console.log(`[API Mirror - appTasks:${portTasks}] Received response from target for ${userReq.method} ${userReq.originalUrl}`);
                return proxyResData;
            },
            proxyErrorHandler: function(err, res, next) {
                //console.error(`[API Mirror - appTasks:${portTasks}] Proxy error for original request to ${res.req.originalUrl}:`, err);
                if (res && !res.headersSent){
                    res.status(500).send('Proxy error');
                } else {
                    next(err);
                }
            }
        }));
    });

    // Serve static files AFTER defining the root route
    appTasks.use(express.static(path.join(__dirname, 'custom_tasks')));
    appTasks.use('/components', express.static(path.join(__dirname, 'components')));
    appTasks.use('/images', express.static(path.join(__dirname, 'images')));
    appTasks.use('/js', express.static(path.join(__dirname, 'js')));
    appTasks.use('/css', express.static(path.join(__dirname, 'css')));

    // --- CUSTOM API (ONLY ON 3001) ---
    appTasks.post('/custom-task', (req, res) => {
        // Custom logic only for port 3001
        res.json({ success: true, message: 'Custom task executed!' });
    });

    appTasks.listen(portTasks, '0.0.0.0', () => {
        //console.log(`Custom server running at http://0.0.0.0:${portTasks}`);
    });
    
    console.log(`Access custom tasks app via http://${NUC_VISION_IP}:${portTasks}`)
}