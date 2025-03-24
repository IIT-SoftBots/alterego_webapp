import { showSyncedPopup } from '../api.js';
import { batteryMonitor } from '../batterymonitor.js';
import { openPopup } from '../utils.js';

export function clickMonitorClose(monitor, unlockMonitor){
    
    monitor.executeCloseFunction();
    unlockMonitor.executeCloseFunction();
}

export class ClickMonitor {
    constructor(websocket, imageElement, maxClicks = 3, timeWindow = 1000) {
        this.websocket = websocket;
        this.imageElement = imageElement;
        this.maxClicks = maxClicks;
        this.timeWindow = timeWindow; // millisecondi
        this.clickTimestamps = [];
        this.init();
    }

    init() {
        // Aggiungi event listener per i click
        this.imageElement.addEventListener('click', this.handleClick.bind(this));
    }

    handleClick(event) {
        const currentTime = Date.now();
        
        // Filtra i timestamp più vecchi del timeWindow
        this.clickTimestamps = this.clickTimestamps.filter(timestamp => 
            currentTime - timestamp <= this.timeWindow
        );
        
        // Aggiungi il nuovo timestamp
        this.clickTimestamps.push(currentTime);
        
        // Controlla se abbiamo raggiunto il numero massimo di click
        if (this.clickTimestamps.length >= this.maxClicks) {
            openPopup();
        }
    }

    executeCloseFunction() {
        // Implementa qui la logica per chiudere l'applicazione
        console.log('Closing Application...');
        
        this.websocket.send(JSON.stringify({ type: 'closeApp' })); // Invia un messaggio di chiusura al server
        
        // Esempio per browser:
        //window.close();
        
        // Rimuovi l'event listener
        this.imageElement.removeEventListener('click', this.handleClick.bind(this));

        // Spegni battery monitor
        batteryMonitor.clearIntervalTimer();
    }
}

export class UnlockClickMonitor {
    constructor(websocket, unlockElement, maxClicks = 3, timeWindow = 1000) {
        this.websocket = websocket;
        this.unlockElement = unlockElement;
        this.maxClicks = maxClicks;
        this.timeWindow = timeWindow; // millisecondi
        this.clickTimestamps = [];
        this.init();
    }

    init() {
        // Aggiungi event listener per i click
        this.unlockElement.addEventListener('click', this.handleClick.bind(this));
        this.unlockElement.style.display = 'block';
    }

    setLocked(val){
        this.guiLocked = val;
    }

    async handleClick(event) {
        const currentTime = Date.now();
        
        // Filtra i timestamp più vecchi del timeWindow
        this.clickTimestamps = this.clickTimestamps.filter(timestamp => 
            currentTime - timestamp <= this.timeWindow
        );
        
        // Aggiungi il nuovo timestamp
        this.clickTimestamps.push(currentTime);
        
        // Controlla se abbiamo raggiunto il numero massimo di click
        if (this.clickTimestamps.length >= this.maxClicks) {

             // First popup - Global warning
            const warnAnsw = await showSyncedPopup(this.websocket, {
                title: 'Unlock Screen',
                text: "Do you want to unlock the screen?",
                icon: 'warning',
                showCancelButton: true,
                allowOutsideClick: false,
                allowEscapeKey: false,
                confirmButtonText: 'Yes'
            });
            
            if (!warnAnsw) return false;

           this.unlockElement.style.display = 'none';
           setTimeout(() => {
                this.unlockElement.style.display = 'block';
            }, 10000);      // Lock again after 10 seconds
        }
    }

    executeCloseFunction() {
        
        // Rimuovi l'event listener
        this.unlockElement.removeEventListener('click', this.handleClick.bind(this));
    }
}
