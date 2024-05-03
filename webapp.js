const express = require('express');
const { exec } = require('child_process');
const bodyParser = require('body-parser');
const app = express();
const port = 3000;


app.use(bodyParser.json()); // for parsing application/json




let sshConnected = false;
const Client = require('ssh2').Client;
// Create a new SSH client instance
const sshClient = new Client();
function connectSSH() {
    if (sshConnected) return;

    return new Promise((resolve, reject) => {


        // Configure the connection parameters
        const connectionParams = {
            host: '192.168.0.50',
            username: 'goldenego-base',
            privateKey: require('fs').readFileSync('/home/goldenego-vision/.ssh/id_rsa')
        };

        // Connect to the SSH server
        sshClient.connect(connectionParams);

        sshClient.on('ready', () => {
            console.log('Connected via SSH!');
            sshConnected = true;
            resolve(); // Resolve the promise
        });

        // Handle errors during the SSH connection process
        sshClient.on('error', (err) => {
            console.error('Error connecting via SSH:', err);
            reject(err); // Reject the promise
        });
    });
}


function execute(command) {
    // Execute the command on the remote server
    sshClient.exec(command, (err, stream) => {
        if (err) throw err;

        stream
            .on('data', (data) => {
                console.log(data.toString());
            })
            .stderr.on('data', (data) => {
                console.error('Command error:', data.toString());
            });
    });
}
// Use the function
app.post('/execute', async (req, res) => {
    const command = req.body.command;
// Call the function
    try {
        if (!sshConnected) {
            await connectSSH();
        }
        // The code here will execute after sshConnected is true
        execute(command);
    } catch (err) {
        console.error('Error:', err);
    }


    res.send('Command execution started');
});

// Add this line to use express.json() middleware
app.use(express.json());
app.use(express.static('/home/goldenego-vision/webapp/alterego_webapp/images'));

app.get('/', (req, res) => {
    res.sendFile('/home/goldenego-vision/webapp/alterego_webapp/main.html');
});

app.post('/grep-command', (req, res) => {
    const command = req.body.command;
    
    sshClient.exec(command, (err, stream) => {
        if (err) {
            console.error(`Error executing command: ${err.message}`);
            res.status(500).json({ error: 'Command execution failed' });
            return;
        }

        let output = '';
        stream.on('data', (data) => {
            output += data.toString();
        });

        stream.on('close', () => {
            res.json({ success: true, output: output });
        });

        stream.stderr.on('data', (data) => {
            console.error('Command error:', data.toString());
        });
    });
});

app.post('/send-videocommand', (req, res) => {
    const { command } = req.body;
    
    exec(`${command}`, (error, stdout, stderr) => {
        if (error) {
            console.error(`Error executing command: ${error.message}`);
            res.status(500).json({ error: 'Command execution failed' });
            return;
        }        
        console.log(`Command output: ${stdout}`);
        res.json({ success: true });
    });
});


app.post('/ping', (req, res) => {
    const ip = req.body.ip;
    exec(`ping -c 1 ${ip}`, (error, stdout, stderr) => {
        if (error) {
            console.log(`error: ${error.message}`);
            res.json({ success: false });
            return;
        }
        if (stderr) {
            console.log(`stderr: ${stderr}`);
            res.json({ success: false });
            return;
        }
        res.json({ success: true });
    });
});

app.listen(port, () => {
    console.log(`Web app running at http://localhost:${port}`);
});


app.use(function (err, req, res, next) {
    console.error(err.stack);
    res.status(500).send('Something broke!');
  });

process.on('uncaughtException', (err) => {
    console.error('Uncaught Exception:', err);
    process.exit(1);
});

process.on('unhandledRejection', (reason, promise) => {
    console.error('Unhandled Rejection:', reason);
    process.exit(1);
});





