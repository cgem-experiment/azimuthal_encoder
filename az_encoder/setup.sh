#!/bin/bash

set -e  # Exit immediately if any command fails

# Step 1: Create the virtual environment under /environments/cgem_az_encoder/
echo "Creating virtual environment under /environments/cgem_az_encoder..."
sudo mkdir -p /environments

sudo python3 -m venv /environments/cgem_az_encoder

sudo chown -R cgem:cgem_control /environments/cgem_az_encoder
sudo chmod -R +775 /environments/cgem_az_encoder

echo "Installing Python dependencies from requirements.txt..."
source /environments/cgem_az_encoder/bin/activate
pip install -r requirements.txt
deactivate

echo "Copying configuration files to /az_encoder..."
sudo mkdir -p /az_encoder
sudo cp configs/*.json /az_encoder/

echo "Copying Python scripts to /az_encoder/scripts..."
sudo mkdir -p /az_encoder/scripts
sudo cp scripts/*.py /az_encoder/scripts/

echo "Copying systemd service files..."
sudo cp systemd/*.service /etc/systemd/system/

echo "Creating /az_encoder and logs directory, setting owners/permissions…"
sudo mkdir -p /az_encoder/logs
sudo chown -R cgem:cgem_control /az_encoder
sudo chmod -R 775       /az_encoder

echo "Reloading systemd and enabling services..."
sudo systemctl daemon-reload
sudo systemctl enable cgem_az_encoder.service

echo "Starting services..."
sudo systemctl start cgem_az_encoder.service

echo "Deployment completed successfully!"
sudo systemctl status cgem_az_encoder.service
