// TEST COPY - Controllo del robot (verifica nodi ROS, doppioni, riavvio protetto). Server: troubleshoot-routes.js

// Funzioni del robot da controllare (nomi dei nodi ROS presi dai launch). Il server decide cosa si può riavviare.
const GROUPS = [
    { id: 'pilot',            label: 'Comandi del pilota',               nodes: ['/socket', '/inbound_data'] },
    { id: 'body_movement',    label: 'Movimento di braccia e testa',     nodes: ['/left/arm_inv_kin_main', '/right/arm_inv_kin_main', '/left/arm_inv_dyn', '/right/arm_inv_dyn', '/head/head_inv_kin', '/pitch_correction'] },
    { id: 'body_activation',  label: 'Motori delle braccia',             nodes: ['/left/qb_manager', '/right/qb_manager'] },
    { id: 'wheels',           label: 'Ruote ed equilibrio',              nodes: ['/wheels/qb_interface_node', '/wheels/lqr', '/wheels/error_handler'] },
    { id: 'imu',              label: 'Sensore di equilibrio',            nodes: ['/imu/Sensor', '/imu/qb_interface_imu_node', '/alterego_state_publisher'] },
    { id: 'navigation',       label: 'Spostamento da solo nella stanza', nodes: ['/lidar', '/amcl', '/map_server', '/map_server_obstacle', '/move_base', '/navigation', '/visualize_robot'] },
    { id: 'face_expressions', label: 'Espressioni del viso',             nodes: ['/face_expressions'] },
    { id: 'face_tracking',    label: 'Guarda e riconosce le persone',    nodes: ['/face_tracker', '/face_recognition'] },
    { id: 'tts',              label: 'Voce (il robot parla)',            nodes: ['/text2speech'] },
    { id: 'stt',              label: 'Ascolto (il robot capisce)',       nodes: ['/speech2text'] }
];
// Funzioni che usano gli stessi nodi delle prove tecniche (bloccate durante una prova)
const SHARED_WITH_TESTS = ['pilot', 'body_movement'];
const WORK_MODE = 3;   // stato "al lavoro" della webapp principale
const STATE_TEXT = { 4: 'è in pausa', 5: 'è sulla base di ricarica', 0: 'non è ancora stato avviato',
                     1: 'si sta accendendo', 2: 'si sta alzando', 6: 'si sta riprendendo da un\'emergenza',
                     7: 'si sta spegnendo', '-1': 'si sta riavviando' };

let lastData = null;
let confirmId = null;          // funzione per cui si sta chiedendo conferma
let restartingId = null;       // funzione in riavvio
let resultMsg = null;          // { kind, title } dopo un riavvio

function render(data, error) {
    if (error) return summary('ko', 'Non riesco a comunicare con il robot.', 'Aspetta qualche secondo e premi "Controlla di nuovo".') + footer(null);
    if (!data) return summary('info', 'Sto controllando il robot…', 'Attendi qualche secondo.') + footer(null);
    if (!data.masterUp) return summary('info', 'Il robot è spento.', 'Accendilo con il pulsante "Accendi robot" e poi controlla di nuovo.') + footer(data);

    const inWork = data.pipelineState === WORK_MODE;
    const rows = GROUPS.map(g => ({ g, st: data.groups[g.id] || { up: false, duplicate: false } }));
    const broken = rows.filter(r => !r.st.up).length;
    const doubled = rows.filter(r => r.st.duplicate).length;

    let html = '';
    if (resultMsg) html += summary(resultMsg.kind, resultMsg.title, '');
    if (doubled > 0) {
        html += summary('warn', doubled === 1 ? 'Una funzione è avviata due volte.' : `${doubled} funzioni sono avviate due volte.`,
                        'Può far comportare il robot in modo strano. Premi "Riavvia" dove c\'è, altrimenti chiama il tecnico.');
    }
    if (!inWork && broken > 0) {
        html += summary('info', `Il robot ${STATE_TEXT[data.pipelineState] || 'non è al lavoro'}.`,
                        'È normale che alcune funzioni siano spente in questo momento.');
    } else if (broken === 0 && doubled === 0) {
        html += summary('ok', 'Tutto funziona correttamente.', '');
    } else if (broken > 0) {
        html += summary('ko', broken === 1 ? 'Una funzione non sta funzionando.' : `${broken} funzioni non stanno funzionando.`,
                        'Premi "Riavvia" dove c\'è. Dove c\'è scritto "Chiama il tecnico", avvisa il tecnico.');
    }
    if (data.testRunning) html += summary('info', 'È in corso una prova tecnica sul robot.', 'Alcune funzioni non si possono riavviare finché la prova non finisce.');

    html += rows.map(({ g, st }) => {
        if (st.duplicate)   return row(g, '#f97316', 'Avviato due volte', '#c2410c',
                                       (inWork && st.fixableDuplicate) ? actionFor(g, data) : `<div class="ts-tech">Chiama il tecnico</div>`, true);
        if (st.up)          return row(g, '#22c55e', 'Funziona', '#15803d', '');
        if (!inWork)        return row(g, '#9ca3af', 'Spento', '#6b7280', '');
        return row(g, '#ef4444', 'Non funziona', '#b91c1c', actionFor(g, data), true);
    }).join('');
    return html + footer(data);
}

function actionFor(g, data) {
    if (restartingId === g.id) return `<div class="ts-busy">Riavvio in corso… attendi circa 20 secondi</div>`;
    if (!data.restartable.includes(g.id)) return `<div class="ts-tech">Chiama il tecnico</div>`;
    if (data.testRunning && SHARED_WITH_TESTS.includes(g.id)) return `<div class="ts-tech">Prova tecnica in corso</div>`;
    if (restartingId) return '';   // un riavvio alla volta
    if (confirmId === g.id) return `
        <div class="ts-confirm">
            <div>Vuoi riavviare questa funzione? Il robot potrebbe fermarsi per qualche secondo.</div>
            <div class="ts-confirm-buttons">
                <button class="ts-yes" data-yes="${g.id}">Sì, riavvia</button>
                <button class="ts-no" data-no="1">Annulla</button>
            </div>
        </div>`;
    return `<button class="ts-restart" data-restart="${g.id}">Riavvia</button>`;
}

function summary(kind, title, detail) {
    return `<div class="ts-summary ${kind}">${title}${detail ? `<small>${detail}</small>` : ''}</div>`;
}
function row(g, dot, text, textColor, action, down) {
    return `<div class="ts-row ${down ? 'down' : ''}">
                <div class="ts-line"><span class="ts-dot" style="background:${dot}"></span>
                    <span class="ts-name">${g.label}</span><span class="ts-state" style="color:${textColor}">${text}</span></div>
                ${action ? `<div class="ts-action">${action}</div>` : ''}
            </div>`;
}
function footer(data) {
    const when = data ? 'Ultimo controllo alle ' + new Date(data.time).toLocaleTimeString('it-IT') : '';
    return `<div class="ts-foot"><span>${when}</span><button class="ts-refresh" id="tsRefresh" ${restartingId ? 'disabled' : ''}>Controlla di nuovo</button></div>`;
}

function draw(error) {
    const box = document.getElementById('tsBody');
    if (!box) return;
    box.innerHTML = render(lastData, error);
    const r = document.getElementById('tsRefresh');
    if (r) r.onclick = () => { resultMsg = null; confirmId = null; refresh(); };
    box.querySelectorAll('[data-restart]').forEach(b => b.onclick = () => { confirmId = b.dataset.restart; resultMsg = null; draw(); });
    box.querySelectorAll('[data-no]').forEach(b => b.onclick = () => { confirmId = null; draw(); });
    box.querySelectorAll('[data-yes]').forEach(b => b.onclick = () => doRestart(b.dataset.yes));
}

async function refresh() {
    lastData = null;
    draw();
    let error = null;
    try {
        const res = await fetch('/api/check');
        const data = await res.json();
        if (data.ok) lastData = data; else error = data.error || 'errore';
    } catch (e) { error = 'server'; }
    draw(error);
}

async function doRestart(id) {
    const g = GROUPS.find(x => x.id === id);
    confirmId = null; restartingId = id; resultMsg = null;
    draw();
    let msg;
    try {
        const res = await fetch('/api/restart/' + encodeURIComponent(id), { method: 'POST' });
        msg = await res.json();
    } catch (e) { msg = { ok: false, message: 'Non riesco a comunicare con il robot. Chiama il tecnico.' }; }
    restartingId = null;
    resultMsg = { kind: msg.ok ? 'ok' : 'ko', title: `"${g.label}": ${msg.message}` };
    await refresh();
}

// Collega il pulsante nel riquadro System Status (chiamata da main.js dopo il caricamento dei componenti)
export function initTroubleshoot() {
    const btn = document.getElementById('debugBtn');
    if (!btn) return;
    btn.addEventListener('click', () => {
        Swal.fire({ title: 'Controllo del robot', html: '<div class="ts-wrap" id="tsBody"></div>',
                    customClass: { popup: 'ts-popup' }, confirmButtonText: 'Chiudi',
                    confirmButtonColor: '#4f6e90', allowOutsideClick: () => !restartingId,
                    didOpen: () => { resultMsg = null; confirmId = null; refresh(); } });
    });
}
