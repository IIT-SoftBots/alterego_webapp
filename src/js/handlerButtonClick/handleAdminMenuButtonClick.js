import { sendCommand, showSyncedPopup, stopBatteryCheck } from '../api.js';
import { LAUNCH_COMMANDS } from '../constants.js';
import { ws } from '../main.js';
import { openPopup } from '../utils.js';

export function clickMonitorClose(monitor, unlockMonitor){
    
    monitor.executeCloseFunction();
    unlockMonitor.executeCloseFunction();
}

export class ClickMonitor {
    constructor(imageElement, maxClicks = 3, timeWindow = 1000) {
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

    async executeCloseFunction() {
        // Implementa qui la logica per chiudere l'applicazione
        console.log('Closing Application...');
               
        // Stop Battery Monitor
        sendCommand(`${LAUNCH_COMMANDS.BATTERY_MONITOR.STOP}`);
        await new Promise(r => setTimeout(r, 1000));
        console.log("Stop Battery Monitoring");

        // Stop checking battery
        stopBatteryCheck();        

        ws.send(JSON.stringify({ type: 'closeApp' })); // Invia un messaggio di chiusura al server
    
        // Esempio per browser:
        //window.close();
        
        // Rimuovi l'event listener
        this.imageElement.removeEventListener('click', this.handleClick.bind(this));

    }
}

export class UnlockClickMonitor {
    constructor(unlockElement, maxClicks = 3, timeWindow = 1000) {
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
            const warnAnsw = await showSyncedPopup({
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
            }, 30000);      // Lock again after 30 seconds
        }
    }

    executeCloseFunction() {
        
        // Rimuovi l'event listener
        this.unlockElement.removeEventListener('click', this.handleClick.bind(this));
    }
}
