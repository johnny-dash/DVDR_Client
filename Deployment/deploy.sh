#!/bin/bash
git clone https://github.com/johnny-dash/DVDR_Client.git
cd DVDR_Client/Deployment
sudo cp superscript /etc/init.d/
sudo chmod 775 superscript
sudo update-rc.d superscript defaults
echo "Deployment finish pls restart the raspberry pi"
