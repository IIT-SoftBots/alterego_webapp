const express = require('express');
const { exec } = require('child_process');
const { spawn } = require("child_process");
const bodyParser = require('body-parser');
const app = express();
const port = 3000;


app.use(bodyParser.json()); // for parsing application/json
const Client = require('ssh2').Client;
// Create a new SSH client instance
const sshClient = new Client();
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
});
// Handle errors during the SSH connection process
sshClient.on('error', (err) => {
    console.error('Error connecting via SSH:', err);
});
const readline = require('readline');
const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

function execute(command) {
    // Execute the command on the remote server
    sshClient.exec(command, (err, stream) => {
        if (err) throw err;

        stream
            // .on('close', (code, signal) => {
            //     console.log('Command execution closed');
            //     sshClient.end();
            //     rl.close();
            // })
            .on('data', (data) => {
                console.log(data.toString());
            })
            .stderr.on('data', (data) => {
                console.error('Command error:', data.toString());
            });
    });
}

app.post('/execute', (req, res) => {
    const command = req.body.command;
    execute(command);
    res.send('Command execution started');
});

// Add this line to use express.json() middleware
app.use(express.json());
app.use(express.static('/home/goldenego-vision/webapp/alterego_webapp/images'));

app.get('/', (req, res) => {
    res.sendFile('/home/goldenego-vision/webapp/alterego_webapp/main.html');
});

// app.post('/send-command', (req, res) => {
//     const { command } = req.body;
    
//     exec(`ssh goldenego-base@192.168.0.50 '${command}'`, (error, stdout, stderr) => {
//         if (error) {
//             console.error(`Send Error executing command: ${error.message}`);
//             res.status(500).json({ error: 'Send Command execution failed' });
//             return;
//         }
        
//         // console.log(`Command output: ${stdout}`);
//         res.json({ success: true });
//     });
// });

// app.post('/send-command', (req, res) => {
//     const { command } = req.body;
//     const ssh = spawn('ssh', ['goldenego-base@192.168.0.50', command]);

//     let stdoutData = '';
//     let stderrData = '';

//     ssh.stdout.on('data', (data) => {
//         stdoutData += data;
//     });

//     ssh.stderr.on('data', (data) => {
//         stderrData += data;
//     });

//     ssh.on('close', (code) => {
//         if (code !== 0) {
//             console.error(`Send Error executing command: ${stderrData}`);
//             res.status(500).json({ error: 'Send Command execution failed' });
//             return;
//         }

//         // console.log(`Command output: ${stdoutData}`);
//         res.json({ success: true });
//     });
// });
app.post('/grep-command', (req, res) => {
    const { command } = req.body;
    
    exec(`${command}`, (error, stdout, stderr) => {
        if (error) {
            console.error(`Grep Error executing command: ${error.message}`);
            res.status(500).json({ error: 'Grep execution failed' });
            return;
        }
        
        res.json({ success: true, output: stdout });
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





