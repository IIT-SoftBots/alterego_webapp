import { UI_STATES } from './constants.js';

// Salva lo stato dei pulsanti nel localStorage
export function saveButtonState(buttonId, state) {
    localStorage.setItem(buttonId, state);
}

// Carica lo stato dei pulsanti dal localStorage
export function loadButtonState(buttonId) {
    return localStorage.getItem(buttonId) === 'true';
}

// Aggiorna l'interfaccia utente in base allo stato
export function updateUI(state) {
    const powerBtn = document.getElementById('powerBtn');
    const startBtn = document.getElementById('startBtn');
    const homeBtn = document.getElementById('homeBtn');
    const powerStatus = document.getElementById('powerStatus');
    const systemStatus = document.getElementById('systemStatus');

    // Aggiorna stato power
    if (state.isPowered) {
        powerBtn.classList.add('on');
        startBtn.disabled = false;
        homeBtn.disabled = false;
        powerStatus.style.backgroundColor = UI_STATES.POWER_ON.color;
        document.querySelector('#powerStatus + span').textContent = UI_STATES.POWER_ON.text;
    } else {
        powerBtn.classList.remove('on');
        startBtn.disabled = true;
        homeBtn.disabled = true;
        powerStatus.style.backgroundColor = UI_STATES.POWER_OFF.color;
        document.querySelector('#powerStatus + span').textContent = UI_STATES.POWER_OFF.text;
    }

    // Aggiorna stato sistema
    if (state.isRunning) {
        startBtn.classList.add('running');
        startBtn.innerHTML = '<i class="fas fa-pause"></i><span>Pause</span>';
        systemStatus.style.backgroundColor = UI_STATES.SYSTEM_RUNNING.color;
        document.querySelector('#systemStatus + span').textContent = UI_STATES.SYSTEM_RUNNING.text;
    } else {
        startBtn.classList.remove('running');
        startBtn.innerHTML = '<i class="fas fa-play"></i><span>Start</span>';
        systemStatus.style.backgroundColor = UI_STATES.SYSTEM_READY.color;
        document.querySelector('#systemStatus + span').textContent = UI_STATES.SYSTEM_READY.text;
    }
}

// Carica i componenti HTML
export async function loadComponent(elementId, componentPath) {
    try {
        const response = await fetch(componentPath);
        const html = await response.text();
        document.getElementById(elementId).innerHTML = html;
    } catch (error) {
        console.error(`Error loading component ${componentPath}:`, error);
    }
}