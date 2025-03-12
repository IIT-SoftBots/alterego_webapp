import { batteryMonitor } from '../batterymonitor.js';
import { openPopup } from '../utils.js';

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
