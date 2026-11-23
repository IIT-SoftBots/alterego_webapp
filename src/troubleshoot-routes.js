// AlterEGO - Controllo robot - COPIA DI PROVA
// Runs next to the main webapp (which is NOT touched) on port 3001.
//  - GET  /api/check        : read-only (rosnode list, pgrep of test launches, main webapp state.json)
//  - POST /api/restart/:id  : restarts ONE whitelisted function, only if every safety rule passes.
// There is no endpoint that accepts a command: every command is fixed in this file.
// Node 10 compatible.

'use strict';
// Troubleshoot routes for the TEST COPY of the webapp (port 3001). Mounted by webapp.js of the copy.
//  - GET  /api/check        : read-only (rosnode list, pgrep, this copy's state.json)
//  - POST /api/restart/:id  : restarts ONE whitelisted function, only if every safety rule passes.
const path = require('path');
const fs = require('fs');
const os = require('os');
const http = require('http');
const childProcess = require('child_process');
const { Client } = require('ssh2');

const PORT = 3000;                                                  // port of the webapp
const STATE_FILE = path.join(__dirname, 'js/state.json');            // state of THIS copy
const SSH_KEY = path.join(os.homedir(), '.ssh/id_rsa');           // same key the main webapp uses
const REMOTE_USER = os.userInfo().username.replace('-vision', '-base');
const ROBOT_NAME = 'robot_adriano';
const LOG_DIR = path.join(__dirname, '..', 'logs');
const WORK_MODE = 3;                                                // main webapp STATE.WORK_MODE

let BASE_IP = '192.168.0.110';
let VISION_IP = '192.168.0.111';
let CONFIG_VERSION = null;                                          // AlterEgoVersion from the main webapp

// ---------------------------------------------------------------------------
// Functions that can be restarted. Anything not listed here can NOT be restarted.
//   host: where the launch runs (same as the main webapp)
//   version: pass AlterEgoVersion:=<version the robot was started with>
//   sharedWithTests: same node names as Adriano's test kits -> blocked while a test runs
// Not restartable on purpose: arm motors (qb_manager), wheels/balance, IMU, navigation.
// ---------------------------------------------------------------------------
const RESTARTABLE = {
    pilot:            { host: 'base',   pkg: 'alterego_robot', launch: 'pilot.launch', version: true, sharedWithTests: true,
                        nodes: ['/socket', '/inbound_data'] },
    body_movement:    { host: 'base',   pkg: 'alterego_robot', launch: 'body_movement.launch', version: true, sharedWithTests: true,
                        nodes: ['/left/arm_inv_kin_main', '/right/arm_inv_kin_main', '/left/arm_inv_dyn', '/right/arm_inv_dyn', '/head/head_inv_kin', '/pitch_correction'] },
    face_expressions: { host: 'base',   pkg: 'alterego_robot', launch: 'face_expressions.launch', version: true,
                        nodes: ['/face_expressions'] },
    // face_tracking.launch on vision starts BOTH face_tracker and face_recognition -> one function
    face_tracking:    { host: 'vision', pkg: 'alterego_face_tracking', launch: 'face_tracking.launch',
                        nodes: ['/face_tracker', '/face_recognition'] },
    tts:              { host: 'vision', pkg: 'alterego_text2speech', launch: 'text2speech_prefetch.launch',
                        nodes: ['/text2speech'] },
    stt:              { host: 'vision', pkg: 'alterego_conversation', launch: 'speech2text_multilang.launch',
                        nodes: ['/speech2text'] }
};

// Functions that are only CHECKED (never restarted from here), needed to spot duplicates.
const CHECK_ONLY = {
    body_activation: { host: 'base', pkg: 'alterego_robot', launch: 'body_activation.launch', nodes: ['/left/qb_manager', '/right/qb_manager'] },
    wheels:          { host: 'base', pkg: 'alterego_robot', launch: 'wheels.launch', nodes: ['/wheels/qb_interface_node', '/wheels/lqr', '/wheels/error_handler'] },
    imu:             { host: 'base', pkg: 'alterego_robot', launch: 'imu.launch', nodes: ['/imu/Sensor', '/imu/qb_interface_imu_node', '/alterego_state_publisher'] },
    // /initialpose is not listed: it sends the start position once and then exits on its own
    navigation:      { host: 'base', pkg: 'alterego_navigation', launch: 'autonomous_nav.launch',
                       nodes: ['/lidar', '/amcl', '/map_server', '/map_server_obstacle', '/move_base', '/navigation', '/visualize_robot'] }
};
const ALL_GROUPS = Object.assign({}, RESTARTABLE, CHECK_ONLY);
// Other launch files that start the same nodes (the main webapp has two speech-to-text launches)
RESTARTABLE.stt.otherLaunches = ['speech2text.launch'];

function rosEnvBase() {
    return 'export ROS_MASTER_URI=http://' + BASE_IP + ':11311 ROS_IP=' + BASE_IP + ' ROS_HOSTNAME=' + BASE_IP +
           ' ROBOT_NAME=' + ROBOT_NAME + '; source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash';
}
function rosEnvVision() {
    return 'export ROS_MASTER_URI=http://' + BASE_IP + ':11311 ROS_IP=' + VISION_IP + ' ROS_HOSTNAME=' + VISION_IP +
           ' ROBOT_NAME=' + ROBOT_NAME + '; source /opt/ros/noetic/setup.bash; source ~/AlterEGO_Adriano/catkin_ws/devel/setup.bash';
}

// ---------------- command runners ----------------
function sshExec(command, timeoutMs) {
    return new Promise(function (resolve, reject) {
        const client = new Client();
        let out = '';
        const timer = setTimeout(function () { client.end(); reject(new Error('timeout')); }, timeoutMs || 20000);
        client.on('ready', function () {
            client.exec('bash -lc ' + JSON.stringify(command), function (err, stream) {
                if (err) { clearTimeout(timer); client.end(); return reject(err); }
                stream.on('data', function (d) { out += d.toString(); });
                stream.stderr.on('data', function () {});
                stream.on('close', function () { clearTimeout(timer); client.end(); resolve(out); });
            });
        });
        client.on('error', function (err) { clearTimeout(timer); reject(err); });
        client.connect({ host: BASE_IP, username: REMOTE_USER, privateKey: fs.readFileSync(SSH_KEY), readyTimeout: 8000 });
    });
}
function localExec(command, timeoutMs) {
    return new Promise(function (resolve) {
        childProcess.exec(command, { shell: '/bin/bash', timeout: timeoutMs || 20000 }, function (err, stdout) { resolve(stdout || ''); });
    });
}
function sleep(ms) { return new Promise(function (r) { setTimeout(r, ms); }); }

// ---------------- check (read-only) ----------------
async function runCheck() {
    const cmd = [
        rosEnvBase(),
        'echo "===NODES==="',
        'timeout 8 rosnode list 2>/dev/null || echo "__MASTER_DOWN__"',
        'echo "===VERSION==="',
        'timeout 5 rosparam get /AlterEgoVersion 2>/dev/null || true',
        'echo "===TESTS==="',
        "pgrep -af 'roslaunch.*(test0|test0_hardware|handover|gestures|trial-gestures)[.]launch' || true",
        PROC_LIST
    ].join('; ');
    const results = await Promise.all([sshExec(cmd), localExec(PROC_LIST)]);
    const out = results[0];
    const pb = parseProcs(part2(out, '===LAUNCHES===')), pv = parseProcs(part2(results[1], '===LAUNCHES==='));
    const procs = { launches: pb.launches.concat(pv.launches), names: pb.names.concat(pv.names) };
    const part = function (a, b) { const s = out.split(a)[1] || ''; return b ? s.split(b)[0] : s; };
    const nodesPart = part('===NODES===', '===VERSION===');
    const version = parseInt(part('===VERSION===', '===TESTS===').trim(), 10);
    const tests = part('===TESTS===', '===LAUNCHES===').split('\n').map(function (s) { return s.trim(); }).filter(Boolean);
    return {
        masterUp: nodesPart.indexOf('__MASTER_DOWN__') === -1,
        nodes: nodesPart.split('\n').map(function (s) { return s.trim(); }).filter(function (s) { return s.charAt(0) === '/'; }),
        runningVersion: isNaN(version) ? null : version,
        testRunning: tests.length > 0,
        pipelineState: readPipelineState(),
        launches: procs.launches,
        names: procs.names
    };
}

// Lists real roslaunch processes and ROS node processes (on base and on vision).
// '[r]' and '[_]' keep the pattern from matching the shell that runs it.
const PROC_LIST = "echo '===LAUNCHES==='; pgrep -af '/opt/ros/noetic/bin/[r]oslaunch ' || true; " +
                  "echo '===PROCS==='; pgrep -af '[_]_name:=' || true";
function part2(text, marker) { return marker + (text.split(marker)[1] || ''); }
function parseProcs(text) {
    const launchText = (text.split('===LAUNCHES===')[1] || '').split('===PROCS===')[0];
    const procText = text.split('===PROCS===')[1] || '';
    const launches = [];
    launchText.split('\n').forEach(function (line) {
        const m = line.match(/\/roslaunch\s+(.*)$/);
        if (!m) return;
        const file = m[1].split(/\s+/).filter(function (t) { return /\.launch$/.test(t); })[0];
        if (file) launches.push(path.basename(file));
    });
    const names = [];
    procText.split('\n').forEach(function (line) {
        const m = line.match(/__name:=(\S+)/);
        if (m) names.push(m[1]);
    });
    return { launches: launches, names: names };
}
function countOf(list, value) { return list.filter(function (x) { return x === value; }).length; }

// Per-function status: running, and whether something is started twice.
function groupStatus(id, c) {
    const g = ALL_GROUPS[id];
    const up = isGroupUp(g, c.nodes);
    const launchFiles = [g.launch].concat(g.otherLaunches || []);
    const launchCount = launchFiles.reduce(function (n, f) { return n + countOf(c.launches, f); }, 0);
    // how many processes are expected for each node base name (left/right share the same name)
    const expected = {};
    g.nodes.forEach(function (n) { const b = n.split('/').pop(); expected[b] = (expected[b] || 0) + 1; });
    const doubleNodes = Object.keys(expected).filter(function (b) { return countOf(c.names, b) > expected[b]; });
    return {
        up: up,
        launchCount: launchCount,
        doubleNodes: doubleNodes,
        duplicate: launchCount > 1 || doubleNodes.length > 0,
        // a function started twice from the SAME launch file is fixed by a restart (all copies are stopped first)
        fixableDuplicate: launchCount > 1 && !!RESTARTABLE[id]
    };
}
function readPipelineState() {
    try { return JSON.parse(fs.readFileSync(STATE_FILE, 'utf8')).pipelineState; } catch (e) { return null; }
}
function isGroupUp(group, nodes) {
    return group.nodes.every(function (n) { return nodes.indexOf('/' + ROBOT_NAME + n) !== -1; });
}

// ---------------- restart ----------------
let restartInProgress = false;

async function restartGroup(id) {
    const g = RESTARTABLE[id];
    if (!g) return { ok: false, message: 'Questa funzione non si può riavviare da qui. Chiama il tecnico.' };

    // 1. Safety checks on fresh data
    const c = await runCheck();
    if (!c.masterUp) return { ok: false, message: 'Il robot è spento. Accendilo con "Accendi robot".' };
    if (c.pipelineState !== WORK_MODE) return { ok: false, message: 'Il robot non è al lavoro: in questo momento non si può riavviare niente.' };
    if (g.sharedWithTests && c.testRunning) return { ok: false, message: 'È in corso una prova tecnica: non si può riavviare questa funzione adesso.' };
    const st = groupStatus(id, c);
    if (st.up && !st.duplicate) return { ok: true, message: 'Questa funzione sta già funzionando.' };
    if (st.duplicate && !st.fixableDuplicate) return { ok: false, message: 'Questa funzione è avviata due volte in un modo che non posso sistemare da qui. Chiama il tecnico.' };

    const version = c.runningVersion || CONFIG_VERSION;
    if (g.version && !version) return { ok: false, message: 'Non riesco a capire la versione del robot. Chiama il tecnico.' };

    // 2. Stop what is left of this function (old roslaunch + surviving nodes). Fixed names only.
    const files = [g.launch].concat(g.otherLaunches || []).map(function (f) { return f.replace('.launch', '[.]launch'); });
    const pattern = "'[r]oslaunch " + g.pkg + ' (' + files.join('|') + ")'";
    const alive = g.nodes.map(function (n) { return '/' + ROBOT_NAME + n; }).filter(function (n) { return c.nodes.indexOf(n) !== -1; });
    const env = g.host === 'base' ? rosEnvBase() : rosEnvVision();
    const stopCmd = env + '; pkill -INT -f ' + pattern + ' || true; sleep 3; ' +
                    (alive.length ? 'timeout 10 rosnode kill ' + alive.join(' ') + ' || true' : 'true');
    if (g.host === 'base') await sshExec(stopCmd, 30000); else await localExec(stopCmd, 30000);
    await sleep(2000);

    // 3. Start it again, detached, exactly like the main webapp does (same launch file)
    const launchCmd = 'roslaunch ' + g.pkg + ' ' + g.launch + (g.version ? ' AlterEgoVersion:=' + version : '');
    const logName = 'troubleshoot-' + id + '.log';
    if (g.host === 'base') {
        await sshExec('mkdir -p /tmp/troubleshoot; setsid nohup bash -c \'' + rosEnvBase() + '; ' + launchCmd +
                      '\' > /tmp/troubleshoot/' + logName + ' 2>&1 < /dev/null &', 20000);
    } else {
        if (!fs.existsSync(LOG_DIR)) fs.mkdirSync(LOG_DIR);
        const fd = fs.openSync(path.join(LOG_DIR, logName), 'a');
        childProcess.spawn('bash', ['-c', rosEnvVision() + '; ' + launchCmd], { detached: true, stdio: ['ignore', fd, fd] }).unref();
    }
    console.log(new Date().toISOString() + ' RESTART ' + id + ' -> ' + launchCmd + ' (on ' + g.host + ')');

    // 4. Wait and check again
    for (let i = 0; i < 6; i++) {
        await sleep(3000);
        const after = await runCheck();
        const s2 = groupStatus(id, after);
        if (s2.up && !s2.duplicate) return { ok: true, message: 'Fatto! Ora funziona di nuovo.' };
    }
    return { ok: false, message: 'Il riavvio non è riuscito. Chiama il tecnico.' };
}

// ---------------- config from main webapp ----------------
function loadConfigFromMainWebapp() {
    http.get('http://127.0.0.1:' + PORT + '/api/config', function (res) {
        let body = '';
        res.on('data', function (c) { body += c; });
        res.on('end', function () {
            try {
                const cfg = JSON.parse(body);
                if (cfg.NUC_BASE_IP) BASE_IP = cfg.NUC_BASE_IP;
                if (cfg.NUC_VISION_IP) VISION_IP = cfg.NUC_VISION_IP;
                if (cfg.AlterEgoVersion) CONFIG_VERSION = cfg.AlterEgoVersion;
            } catch (e) {}
            console.log('Base ' + BASE_IP + ', vision ' + VISION_IP + ', config version ' + CONFIG_VERSION);
        });
    }).on('error', function () { console.log('Main webapp config not reachable, using defaults'); });
}

let configLoaded = false;

// ---------------- HTTP ----------------
module.exports = function mountTroubleshoot(app) {

app.get('/api/check', async function (req, res) {
    if (!configLoaded) { configLoaded = true; loadConfigFromMainWebapp(); await sleep(500); }
    try {
        const c = await runCheck();
        const groups = {};
        if (c.masterUp) Object.keys(ALL_GROUPS).forEach(function (id) { groups[id] = groupStatus(id, c); });
        res.json({ ok: true, robotName: ROBOT_NAME, masterUp: c.masterUp, nodes: c.nodes, testRunning: c.testRunning, groups: groups,
                   pipelineState: c.pipelineState, restartable: Object.keys(RESTARTABLE),
                   restartInProgress: restartInProgress, time: new Date().toISOString() });
    } catch (err) {
        res.status(500).json({ ok: false, error: 'Cannot reach base NUC: ' + err.message });
    }
});

app.post('/api/restart/:id', async function (req, res) {
    if (!configLoaded) { configLoaded = true; loadConfigFromMainWebapp(); await sleep(500); }
    if (restartInProgress) return res.json({ ok: false, message: 'C\'è già un riavvio in corso. Aspetta che finisca.' });
    restartInProgress = true;
    try {
        res.json(await restartGroup(req.params.id));
    } catch (err) {
        console.error('Restart error:', err);
        res.json({ ok: false, message: 'Il riavvio non è riuscito. Chiama il tecnico.' });
    } finally {
        restartInProgress = false;
    }
});

};
