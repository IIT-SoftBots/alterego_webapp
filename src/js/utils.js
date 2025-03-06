import { STATE, UI_STATES } from './constants.js';

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

    const mainBtn = document.getElementById('mainBtn');
    const secondBtn = document.getElementById('secondBtn');
    const powerStatus = document.getElementById('powerStatus');
    const systemStatus = document.getElementById('systemStatus');

    // Aggiorna stato power
    if (state.isPowered) {
        powerStatus.style.backgroundColor = UI_STATES.POWER_ON.color;
        document.querySelector('#powerStatus + span').textContent = UI_STATES.POWER_ON.text;
    } else {
        powerStatus.style.backgroundColor = UI_STATES.POWER_OFF.color;
        document.querySelector('#powerStatus + span').textContent = UI_STATES.POWER_OFF.text;
    }

    // Aggiorna stato sistema
    if (state.isRunning) {
        systemStatus.style.backgroundColor = UI_STATES.SYSTEM_RUNNING.color;
        document.querySelector('#systemStatus + span').textContent = UI_STATES.SYSTEM_RUNNING.text;
    } else {
        systemStatus.style.backgroundColor = UI_STATES.SYSTEM_READY.color;
        document.querySelector('#systemStatus + span').textContent = UI_STATES.SYSTEM_READY.text;
    }
    
    // Aggiorna stato dei pulsanti
    switch(state.pipelineState){
        case STATE.INIT:
            mainBtn.disabled = false;        
            secondBtn.disabled = false;        
            updateMainBtn('start_robot');
            updateSecondBtn('power_off');
        case STATE.ACTIVATE_ROBOT:    
            mainBtn.disabled = true;  
            secondBtn.disabled = true;      
            break;
        case STATE.STAND_UP:
            mainBtn.disabled = true;
            secondBtn.disabled = true;        
            break;
        case STATE.WORK_MODE:
            mainBtn.disabled = false;
            secondBtn.disabled = false;        
            updateMainBtn('pause');
            updateSecondBtn('home');
            break;
        case STATE.PAUSED:
            mainBtn.disabled = false;        
            secondBtn.disabled = false;        
            updateMainBtn('play');
            break;
        case STATE.DOCKED:
            mainBtn.disabled = false;        
            secondBtn.disabled = false;        
            updateMainBtn('start_robot');
            updateSecondBtn('power_off');
            break;    
        case STATE.RECOVERY_FROM_EMERGENCY:
            mainBtn.disabled = false;        
            secondBtn.disabled = false;        
            updateMainBtn('start_robot');
            updateSecondBtn('power_off');
            break;
        default:
            break;
    }
}

function updateMainBtn(show){

    const mainBtn = document.getElementById('mainBtn');

    if (show == 'start_robot'){
        mainBtn.innerHTML = '<i class="fas fa-circle-play"></i><span>Start Robot</span>';        
        mainBtn.classList.remove('play');
        mainBtn.classList.remove('pause');
    }
    else if (show == 'pause'){
        mainBtn.innerHTML = '<i class="fas fa-pause"></i><span>Pause</span>';
        mainBtn.classList.add('play'); 
        mainBtn.classList.remove('pause');       
    }
    else if (show == 'play'){
        mainBtn.innerHTML = '<i class="fas fa-play"></i><span>Play</span>';
        mainBtn.classList.add('pause');
        mainBtn.classList.remove('play');
    }
}

function updateSecondBtn(show){

    const secondBtn = document.getElementById('secondBtn');

    if (show == 'power_off'){
        secondBtn.innerHTML = '<i class="fas fa-power-off"></i><span>Power Off</span>';  
        secondBtn.classList.remove('home');      
    }
    else if (show == 'home'){
        secondBtn.innerHTML = '<i class="fas fa-home"></i><span>Home</span>';
        secondBtn.classList.add('home');        
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

export function updateBatteryGraphics(isCharging, batteryLevel){
    const batteryIcon = document.getElementById('batteryIcon');

    if (isCharging){
        batteryIcon.innerHTML = "<i class=\"fa fa-charging-station\"></i>";
    }
    else {
        if (batteryLevel > 75){          //75-100%
            batteryIcon.innerHTML = "<i class=\"fa fa-battery-full\"></i>";
        }
        else {
            if (batteryLevel > 50){      //50-75%
                batteryIcon.innerHTML = "<i class=\"fa fa-battery-three-quarters\"></i>";
            }
            else {
                if (batteryLevel > 25){  //25-50%
                    batteryIcon.innerHTML = "<i class=\"fa fa-battery-half\"></i>";
                }
                else {                  //0-25%
                    batteryIcon.innerHTML = "<i class=\"fa fa-battery-quarter\"></i>";
                }
            }   
        }
    }    
}

export function openPopup() {
    document.getElementById('popupOverlay').style.display = 'block';
    document.getElementById('popup').style.display = 'block';
}

// Funzione per l'azione Settings
export function settingsAction() {
    // Esempio di azione: mostra un messaggio
    alert('Settings... TODO');
    closeAdminMenu(); // Chiude il popup dopo l'azione
}

// Funzione per chiudere il popup
export function closeAdminMenu() {
    document.getElementById('popupOverlay').style.display = 'none';
    document.getElementById('popup').style.display = 'none';
}
