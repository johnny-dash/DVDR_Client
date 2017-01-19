#!/bin/sh

sudo ping -c4 10.110.110.254 > /dev/null

while true
do
	if [ $? = 0 ]
	then
		echo "Wifi has connected"
		cd /home/pi/Desktop/MQTT/client
		sudo python asynchronous.py
		break
	else
		echo "Wifi has not connected"
	fi
	sleep 3
done

