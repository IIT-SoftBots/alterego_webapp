const express = require('express');
const { exec } = require('child_process');

const app = express();
const port = 3000;
// Add this line to use express.json() middleware
app.use(express.json());
app.use(express.static('/home/goldenego-vision/webapp/alterego_webapp/images'));

app.get('/', (req, res) => {
    res.sendFile('/home/goldenego-vision/webapp/alterego_webapp/main.html');
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