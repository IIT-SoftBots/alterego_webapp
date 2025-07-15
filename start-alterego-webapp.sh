#!/bin/bash

# 1. Set up application directory
APP_DIR="/home/alterego-vision/AlterEGO_v2/catkin_ws/src/alterego_webapp"

# 2. Correctly initialize NVM (Node Version Manager) to find the right Node.js version.
# This is the standard and most reliable way to use NVM in scripts.
export NVM_DIR="$HOME/.nvm"
if [ -s "$NVM_DIR/nvm.sh" ]; then
  . "$NVM_DIR/nvm.sh"
else
  # If NVM is not found, we can't continue.
  exit 1
fi

# 3. Explicitly load environment variables from the .env file.
if [ -f "$APP_DIR/.env" ]; then
    export $(grep -v '^#' "$APP_DIR/.env" | xargs)
fi

# 4. Dynamically find the path to the executables using 'which'.
# After loading NVM, 'which node' will now point to the correct (newer) version.
NODE_EXEC=$(which node)
FIREFOX_EXEC=$(which firefox)

# 5. Check if the executables were found. If not, exit.
if [ -z "$NODE_EXEC" ]; then
    notify-send "AlterEgo WebApp Error" "Node.js executable not found."
    exit 1
fi
if [ -z "$FIREFOX_EXEC" ]; then
    notify-send "AlterEgo WebApp Error" "Firefox executable not found."
    exit 1
fi

# 6. Set other environment variables and start the application.
export DISPLAY=:0
notify-send "AlterEgo WebApp" "Starting up, please wait..."

# 7. Change to the app directory and start the Node.js server.
# The --experimental-specifier-resolution=node flag will now be understood by the correct Node version.
# We redirect output to /dev/null to prevent it from creating files or showing in logs.
cd "$APP_DIR" || exit
"$NODE_EXEC" --experimental-specifier-resolution=node src/webapp.js > /dev/null 2>&1 &

# 8. Wait intelligently for the server to be ready on port 3000.
attempts=0
max_attempts=15 # Wait for a maximum of 15 seconds

while ! nc -z localhost 3000 && [ $attempts -lt $max_attempts ]; do
    sleep 1
    attempts=$((attempts+1))
done

if [ $attempts -eq $max_attempts ]; then
    notify-send "AlterEgo WebApp Error" "Server failed to start on port 3000."
    exit 1
fi

# 9. Start Firefox in kiosk mode now that the server is confirmed to be running.
"$FIREFOX_EXEC" --kiosk http://localhost:3000
