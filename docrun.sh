#!/bin/bash
# This script starts the container in detached (background) mode.

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo " "
echo "Starting the container in the background..."
docker-compose up -d

echo " "
echo -e "${GREEN}Container is running.${NC}"
echo -e "${YELLOW}You can now run './docshell.sh' to open a shell inside the container.${NC}"
echo -e "${YELLOW}You can now run './docstop.sh' to stop the container.${NC}"