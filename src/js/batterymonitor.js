
export class BatteryMonitor {
    static instance;

    static getInstance() {
        if (!BatteryMonitor.instance) {
            BatteryMonitor.instance = new BatteryMonitor();
        }
        return BatteryMonitor.instance;
    }

    constructor() {
        
        this.powerAlert = false;
        this.isCharging = false;
        this.needCharge = false;
        this.batteryLevel = 0;

        this.oldPowerAlertTrigger = -1;
        this.isSetOldPowerAlertTrigger = false;
        this.oldNeedChargeTrigger = -1;
        this.isSetOldNeedChargeTrigger = false;
        this.firstNeedCharge = false;
        this.shouldAutoRestart = false;
        this.errorCounter = 0;

        this.batteryInterval = [];
        this.isTimerSet = false;
        this.websocket = 0;
        this.fsmState = 0;  
    }

    start(ws, initState){

        this.websocket = ws;
        this.fsmState = initState;
    }   

    getState(){
        return this.fsmState;
    }

    updateFSMState(val){
        this.fsmState = val;
    }

    timerIsSet() {
        return this.isTimerSet;
    }

    updateInterval(int){
        this.batteryInterval = int;
        this.isTimerSet = true;
    }

    clearIntervalTimer(){
        if (this.batteryInterval){
            clearInterval(this.batteryInterval);
            this.isTimerSet = false;
            this.clearOldPowerAlertTrigger();
            this.clearOldNeedChargeTrigger();
        }
    }

    checkPowerAlertTrigger(){
        // Trigger should be detected and handled on time
        if (this.isSetOldPowerAlertTrigger && this.oldPowerAlertTrigger != this.powerAlert){
            this.oldPowerAlertTrigger = this.powerAlert;
            return true;
        }
        return false;
    }
    
    checkNeedChargeTrigger(){
        // Trigger should be detected and handled on time
        if (this.isSetOldNeedChargeTrigger && this.oldNeedChargeTrigger != this.needCharge){
            this.oldNeedChargeTrigger = this.needCharge;
            return true;
        }
        return false;
    }

    getPowerAlert() {
        return this.powerAlert;         
    } 

    getNeedCharge() {
        return this.needCharge;         
    } 

    clearOldPowerAlertTrigger(){
        this.oldPowerAlertTrigger = [];
        this.isSetOldPowerAlertTrigger = false;
    }

    clearOldNeedChargeTrigger(){
        this.oldNeedChargeTrigger = [];
        this.isSetOldNeedChargeTrigger = false;
    }

    updateState(pA, iC, nC, bL){

        if (!this.isTimerSet){
            // It's first message, update oldPowerAlert
            this.oldPowerAlertTrigger = pA;
            this.isSetOldPowerAlertTrigger = true;
            this.oldNeedChargeTrigger = nC;
            this.isSetOldNeedChargeTrigger = true;
        }

        // Update with new values
        this.powerAlert = pA;
        this.isCharging = iC;
        this.needCharge = nC;
        this.batteryLevel = bL;
    }
    
    getErrorCounter(){
        return this.errorCounter;
    }

    updateErrorCounter(){
        this.errorCounter++;
    }

    resetErrorCounter(){
        this.errorCounter = 0;
    }

    getFirstNeedCharge(){
        return this.firstNeedCharge;
    }

    setFirstNeedCharge(val){
        this.firstNeedCharge = val;
    }

    getShouldAutoRestart(){
        return this.shouldAutoRestart;
    }

    setShouldAutoRestart(val){
        this.shouldAutoRestart = val;
    }
}

export const batteryMonitor = BatteryMonitor.getInstance();