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
│   │   ├── workflow.js     # Functions to handle the whole workflow
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


sudo apt install nodejs npm



## Usage
To start the application, open `src/index.html` in a web browser. The application will load the necessary resources and display the control panel interface.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.
-- procedura per il setup della webapp

```
sudo apt update
sudo apt install nginx -y
```

- apri il file
```
sudo nano /etc/nginx/sites-available/default
```

- commenta tutto 
- aggiungi questo

```
server {
    listen 80;
    server_name localhost;

    location / {
        proxy_pass http://localhost:3000; # Cambia con la porta della tua web app
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    }
}
```


riavvia
```
sudo systemctl restart nginx
```

---------------------------------------------------- 
Crea il seguente file:
```
nano ~/.local/share/applications/webapp.desktop
```
Incolla questo codice:
```
[Desktop Entry]
Type=Application
Name=WebApp
Exec=firefox --kiosk http://localhost:3000
Icon=firefox
Terminal=false
Categories=Utility;
```

Lanciala da Applicazioni oppure con:
```
gtk-launch webapp
```


- aggiungi al bashrc:

```
source ~/AlterEGO_v2/EGO_GUI/usefull_scripts.sh
```
