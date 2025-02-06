import React, { useState } from 'react';
import { Power, Settings, AlertCircle, Play, Pause, Save, RefreshCw } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Alert, AlertDescription } from '@/components/ui/alert';

const RobotDashboard = () => {
  const [isPowered, setIsPowered] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [showAlert, setShowAlert] = useState(false);

  const handlePowerToggle = () => {
    setIsPowered(!isPowered);
    if (!isPowered) {
      setShowAlert(true);
      setTimeout(() => setShowAlert(false), 3000);
    } else {
      setIsRunning(false);
    }
  };

  return (
    <div className="w-full max-w-4xl mx-auto p-6">
      <Card className="bg-gray-50">
        <CardHeader className="space-y-1">
          <CardTitle className="text-2xl font-bold text-center">Robot Control Panel</CardTitle>
        </CardHeader>
        <CardContent>
          {showAlert && (
            <Alert className="mb-4 bg-green-100">
              <AlertCircle className="h-4 w-4" />
              <AlertDescription>
                Robot inizializzato con successo
              </AlertDescription>
            </Alert>
          )}
          
          <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
            {/* Bottone di Accensione Principale */}
            <button
              onClick={handlePowerToggle}
              className={`p-6 rounded-lg flex flex-col items-center justify-center gap-2 transition-all ${
                isPowered ? 'bg-green-500 hover:bg-green-600' : 'bg-red-500 hover:bg-red-600'
              } text-white`}
            >
              <Power className="h-8 w-8" />
              <span className="font-medium">{isPowered ? 'Spegni' : 'Accendi'}</span>
            </button>

            {/* Bottone Start/Stop */}
            <button
              onClick={() => setIsRunning(!isRunning)}
              disabled={!isPowered}
              className={`p-6 rounded-lg flex flex-col items-center justify-center gap-2 ${
                !isPowered
                  ? 'bg-gray-300 cursor-not-allowed'
                  : isRunning
                  ? 'bg-yellow-500 hover:bg-yellow-600'
                  : 'bg-blue-500 hover:bg-blue-600'
              } text-white`}
            >
              {isRunning ? <Pause className="h-8 w-8" /> : <Play className="h-8 w-8" />}
              <span className="font-medium">{isRunning ? 'Pausa' : 'Avvia'}</span>
            </button>

            {/* Bottone Configurazione */}
            <button
              disabled={!isPowered}
              className={`p-6 rounded-lg flex flex-col items-center justify-center gap-2 ${
                !isPowered
                  ? 'bg-gray-300 cursor-not-allowed'
                  : 'bg-purple-500 hover:bg-purple-600'
              } text-white`}
            >
              <Settings className="h-8 w-8" />
              <span className="font-medium">Configurazione</span>
            </button>

            {/* Bottone Salva Impostazioni */}
            <button
              disabled={!isPowered}
              className={`p-6 rounded-lg flex flex-col items-center justify-center gap-2 ${
                !isPowered
                  ? 'bg-gray-300 cursor-not-allowed'
                  : 'bg-indigo-500 hover:bg-indigo-600'
              } text-white`}
            >
              <Save className="h-8 w-8" />
              <span className="font-medium">Salva Config</span>
            </button>

            {/* Bottone Reset */}
            <button
              disabled={!isPowered}
              className={`p-6 rounded-lg flex flex-col items-center justify-center gap-2 ${
                !isPowered
                  ? 'bg-gray-300 cursor-not-allowed'
                  : 'bg-orange-500 hover:bg-orange-600'
              } text-white`}
            >
              <RefreshCw className="h-8 w-8" />
              <span className="font-medium">Reset</span>
            </button>
          </div>

          {/* Status Panel */}
          <div className="mt-6 p-4 bg-white rounded-lg shadow">
            <h3 className="font-semibold mb-2">Stato Sistema</h3>
            <div className="grid grid-cols-2 gap-4">
              <div className="flex items-center gap-2">
                <div className={`w-3 h-3 rounded-full ${isPowered ? 'bg-green-500' : 'bg-red-500'}`} />
                <span>Alimentazione: {isPowered ? 'Attiva' : 'Disattiva'}</span>
              </div>
              <div className="flex items-center gap-2">
                <div className={`w-3 h-3 rounded-full ${isRunning ? 'bg-green-500' : 'bg-yellow-500'}`} />
                <span>Stato: {isRunning ? 'In esecuzione' : 'In attesa'}</span>
              </div>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
};

export default RobotDashboard;