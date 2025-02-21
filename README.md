# AlterEGO Web Application

## Overview
The AlterEGO Web Application is a web-based control panel designed to manage and monitor the AlterEGO robotic system. This application provides a user-friendly interface for interacting with the robot's functionalities.

## Project Structure
```
alterego-webapp
├── src
│   ├── index.html          # Main HTML entry point
│   ├── js
│   │   ├── constants.js    # Constants used throughout the application
│   │   ├── utils.js        # Utility functions for data manipulation
│   │   ├── api.js          # Functions for making API calls
│   │   └── main.js         # Main JavaScript file for initialization
│   ├── css
│   │   └── main.css        # Styles for the web application
│   └── components
│       ├── status-panel.html # HTML structure for the status panel
│       └── button-grid.html  # HTML structure for the button grid
├── package.json            # npm configuration file
└── README.md               # Project documentation
```

## Setup Instructions
1. Clone the repository:
   ```
   git clone <repository-url>
   ```
2. Navigate to the project directory:
   ```
   cd alterego-webapp
   ```
3. Install the dependencies:
   ```
   npm install
   ```

## Usage
To start the application, open `src/index.html` in a web browser. The application will load the necessary resources and display the control panel interface.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

---------------------------------------------------- Funge ma a schermo intero
nano ~/.local/share/applications/webapp.desktop

Incolla questo codice:

[Desktop Entry]
Type=Application
Name=WebApp
Exec=firefox --kiosk http://192.168.0.71
Icon=firefox
Terminal=false
Categories=Utility;

Lanciala da Applicazioni oppure con:
gtk-launch webapp


-- aggiungi al bashrc:
alias launch_webapp='node /home/alterego-vision/AlterEGO_v2/catkin_ws/src/alterego_webapp/src/webapp.js '
alias src_bashrc='source ~/.bashrc'


echo "usefull aliases:"
echo "src_bashrc"
echo "launch_webapp"
echo "gtk-launch webapp"
<!-- 
-- attivo startup 
mkdir -p ~/.config/autostart


--crea il file
sudo nano ~/.config/autostart/alterego-apps.desktop

-- copia
[Desktop Entry]
Type=Application
Name=AlterEGO Apps
Comment=Start AlterEGO WebApp and GUI
Exec=bash -c 'source ~/.bashrc && launch_webapp && gtk-launch webapp'
Terminal=false
X-GNOME-Autostart-enabled=true

--rendi eseguibile
sudo chmod +x ~/.config/autostart/alterego-apps.desktop

--test
bash -c 'source ~/.bashrc && launch_webapp && gtk-launch webapp' --> Fallito

-- creo file 
sudo nano ~/.config/autostart/start-alterego.sh
-- aggiungi 

#!/bin/bash
source /home/alterego-vision/.bashrc
sleep 2  # Wait for system to fully initialize
node /home/alterego-vision/AlterEGO_v2/catkin_ws/src/alterego_webapp/src/webapp.js &
sleep 5  # Wait for webapp to start
gtk-launch webapp

-- rendi eseguibile 
chmod +x ~/.config/autostart/start-alterego.sh


--crea 
sudo nano ~/.config/autostart/alterego-apps.desktop

-- aggiungi 

[Desktop Entry]
Type=Application
Name=AlterEGO Apps
Comment=Start AlterEGO WebApp and GUI
Exec=/home/alterego-vision/.config/autostart/start-alterego.sh
Terminal=false
X-GNOME-Autostart-enabled=true

---------------------------------------------------- funge bene con barra laterale
sudo apt install nodejs npm -y
npm install -g nativefier


nativefier --name "WebApp" --single-instance --disable-dev-tools --disable-context-menu http://192.168.0.71

./WebApp-linux-x64/WebApp

(Opzionale) Per aggiungerla al menu delle applicazioni:

Copia la cartella in /opt/
Crea un file .desktop in ~/.local/share/applications/


---------------------------------------------------- apre motore di ricerca
nano ~/.config/autostart-webapp.sh


#!/bin/bash
source ~/.bashrc  # Carica gli alias
webapp &          # Esegue il primo alias in background
sleep 5           # Aspetta un po'
webapp_gui &      # Esegue il secondo alias in background

chmod +x ~/.config/autostart-webapp.sh
