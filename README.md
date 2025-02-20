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

---------------------------------------------------- Funge ma a cshermo intero
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


---------------------------------------------------- funge bene
sudo apt install nodejs npm -y
npm install -g nativefier


nativefier --name "WebApp" --single-instance --disable-dev-tools --disable-context-menu http://192.168.0.71

./WebApp-linux-x64/WebApp

(Opzionale) Per aggiungerla al menu delle applicazioni:

Copia la cartella in /opt/
Crea un file .desktop in ~/.local/share/applications/


----------------------------------------------------
nano ~/.config/autostart-webapp.sh


#!/bin/bash
source ~/.bashrc  # Carica gli alias
webapp &          # Esegue il primo alias in background
sleep 5           # Aspetta un po'
webapp_gui &      # Esegue il secondo alias in background

chmod +x ~/.config/autostart-webapp.sh
