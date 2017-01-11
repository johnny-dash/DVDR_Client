#!/usr/bin/env python
#
# GrovePi Example for using the Grove Light Sensor and the LED together to turn the LED On and OFF if the background light is greater than a threshold.
# Modules:
# 	http://www.seeedstudio.com/wiki/Grove_-_Light_Sensor
# 	http://www.seeedstudio.com/wiki/Grove_-_LED_Socket_Kit
#
# The GrovePi connects the Raspberry Pi and Grove sensors.  You can learn more about GrovePi here:  http://www.dexterindustries.com/GrovePi
#
# Have a question about this example?  Ask on the forums here:  http://www.dexterindustries.com/forum/?forum=grovepi
#
'''
## License

The MIT License (MIT)

GrovePi for the Raspberry Pi: an open source platform for connecting Grove Sensors to the Raspberry Pi.
Copyright (C) 2015  Dexter Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

import time
import grovepi
import paho.mqtt.client as mqtt
import threading
import multiprocessing
import xml.etree.ElementTree as ET

#get device serial number
from GrovepiSerial import getserial



#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------          Inital Configuration                     ----------------------------
#----------------------------------------------------------------------------------------------------------------------

#Process Pool
pool = []

# SIG,NC,VCC,GND
#air_sensor = 0                  # Connect the Grove Air Sensor to analog port A0
#dht_sensor_port = 8		 # Connect the DHt sensor to port 8


#Device Serial Number
cpuserial = "0000000000000000"

#set up serial number
myserial = getserial()

#inital xml configuration file
global root
global task
global et
try:
        et = ET.parse('config.xml')
        root = et.getroot()
        task = root.find('task')

        #the flag that if the device has been registered
        register_flag = root.find('register_flag').text
except ImportError:
        print("Import Error")
        
#Topic setting
pub_air = myserial + ':Air_Quality:A0'                          #generate unique topic for air quality sensor
pub_temp = myserial + ':temperature&humidity:D8'                #generate unique topic for temperature&humidity sensor


#MQTT Server address
server = '130.56.250.107'


sub_topic = '#' #subscribe all topics
pub_register = 'new device' #publish topic for device configuration


#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------          Sensor function                     ---------------------------------
#----------------------------------------------------------------------------------------------------------------------

def air_quality_sensor(frequence,port):
        while True:
                try:
                        #format port
                        air_sensor = int(port[1])
                        #grovepi configuration
                        #grovepi.pinMode(air_sensor,"INPUT")
                        client = mqtt.Client()
                        client.connect(server, 1883, 60)
                        r = grovepi.analogRead(air_sensor)
                        client.publish(pub_air, "Air_qulity_SensorA0@Raspberry Pi No.1: Sensor_value = %d" % r, 1)
                        time.sleep(frequence)

                except IOError:
                        print ("Error") 

def temperature_humidity_sensor(frequence,port):
        while True:
                try:
                        #format port
                        dht_sensor_port = int(port[1])
                        client = mqtt.Client()
                        client.connect(server, 1883, 60)
                        #sensor config and value read
                        [ temp,hum ] = grovepi.dht(dht_sensor_port,1)
                        client.publish(pub_temp, "temperature&humidity_SensorD8@Raspberry Pi No.1: Sensor_value = %d" % temp, 1)
                        time.sleep(frequence)
                
                except (IOError,TypeError) as e:
                        print("Error")

def light_sensor(frequence, port):
        while True:
                try:
                        #format port
                        light_sensor = int(port[1])
                        client = mqtt.Client()
                        client.connect(server, 1883, 60)
                        sensor_value = grovepi.analogRead(light_sensor)
                        client.publish(pub_temp,"Air_qulity_SensorA0@Raspberry Pi No.1: Sensor_value = %d" % sensor_value, 1)
                        time.sleep(frequence)

                except (IOError, TypeError) as e:
                        print("Error")
                        

#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------          Process Manager                     ---------------------------------
#----------------------------------------------------------------------------------------------------------------------
def createtsk(target, name, tsk_freq, tsk_port):
        #start the corresponding function as process
        thr = multiprocessing.Process(target = target, args=(float(tsk_freq),tsk_port,))      #create the thread to run the corresponding sensor function
        thr.start()
        #append the process into process pool
        pool.append((name,thr))

def stoptsk(name):
        #iterate to find exsisted process
        for Key, Value in pool:
                if(Key == name):
                        #kill the process and remove from pool
                        Value.terminate()
                        pool.remove((Key,Value))

def updatetsk(target, name, tsk_freq, tsk_port):
        #iterate to find exsisted process
        for Key, Value in pool:
                if(Key == name):
                        #kill the process and remove from pool
                        Value.terminate()
                        pool.remove((Key,Value))
                                
        #start the corresponding function as process
        thr = multiprocessing.Process(target = target, args=(float(tsk_freq),tsk_port,))      #create the thread to run the corresponding sensor function
        thr.start()
        #append the process into process pool
        pool.append((name,thr))

#action filter to take different actions
def tskAction(function, action, tsk_tag, set_sensor, set_fre, set_port, set_enroll, set_tskname ):
        if(action == 'start'):
                #create thread
                createtsk(function,tsk_tag,set_fre , set_port)
                #add task into xml file
                Add_new_tsk(set_tskname, set_sensor, action, set_fre, set_port, set_enroll)
                
        elif(action == 'stop'):
                #delete the thread
                stoptsk(tsk_tag)
                
        elif(action == 'restart'):
                #restart stop task
                createtsk(function,tsk_tag,set_fre , set_port)

        elif(action == 'delete'):
                #delete the thread and delete task info in xml
                stoptsk(tsk_tag)
                Remove_tsk(set_tskname)
                
        elif(action == 'update'):
                #update the thread and update the xml config file
                updatetsk(function,tsk_tag, set_fre, set_port)
                Remove_tsk(set_tskname)
                Add_new_tsk(set_tskname, set_sensor, action, set_fre, set_port, set_enroll)
                
#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------               XML                      ---------------------------------------
#----------------------------------------------------------------------------------------------------------------------
def Add_new_tsk(tsk_name, sensor, status, frequency, port, enrollment):
        tsk = ET.SubElement(task, tsk_name)      
        tsk_sensor = ET.SubElement(tsk, "sensor")
        tsk_sensor.text = sensor
        tsk_status = ET.SubElement(tsk, "status")
        tsk_status.text = status
        tsk_frequence = ET.SubElement(tsk, "frequency")
        tsk_frequence.text = frequency
        tsk_port = ET.SubElement(tsk, "port")
        tsk_port.text = port
        tsk_enrollment = ET.SubElement(tsk, "enrollment")
        tsk_enrollment.text = enrollment
        ET.dump(task)
        et.write('config.xml')

def Remove_tsk(tsk_name):
        rm_tsk = task.find(tsk_name)
        task.remove(rm_tsk)
        et.write('config.xml')
        
#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------               MQTT                     ---------------------------------------
#----------------------------------------------------------------------------------------------------------------------
def on_connect(client, data, flags, rc):
	print 'Connect with the result code ' + str(rc)         #return connection status
        
	#subscribe message from sensor
	client.subscribe(sub_topic, 1)

def on_message(client, data, msg):
        device = str(msg.topic).split(':')[0]
        if device == myserial:
                topic = str(msg.topic).split(':')[1]
                if topic == 'task':
                        global frequence
                        #phrase the message for task(may become a independent function)
                        info = str(msg.payload).split(':')
                        sensor = info[0];
                        status = info[1];
                        frequency = info[2];
                        port = info[3]
                        enrollment = info[4]
                        tsk_name = info[5]
                        
                        #check the output
                        print("sensor:" + sensor + " status:" + status + " frequency:" + frequency + " port:" + port + " enrollment:" + enrollment + " tsk name:" + tsk_name)

                        #filter 
                        if(sensor == 'air_quality'):
                                tskAction(air_quality_sensor, status, 'air', sensor, frequency, port, enrollment, tsk_name)                                
                                                                        
                        elif(sensor == 'temperature_humidity'):
                                tskAction(temperature_humidity_sensor, status, 'temp', sensor, frequency, port, enrollment, tsk_name)
                                                                
                        elif(sensor == 'light'):
                                tskAction(light_sensor, status, 'light', sensor, frequency, port, enrollment, tsk_name)
                                                               
                        else:
                                print('no corresponding task on client')
                elif topic == 'device':
                        global register_flag
                        if str(msg.payload) == "record":
                                print("device has been detected")
                                register_flag = True
                                root.find('register_flag').text = "True"
                        elif str(msg.payload) == "unregistered":
                                print("device has been unregistered")
                                register_flag = False
                                root.find('register_flag').text = "False"
                        else:
                                print('device config fail')         
                       
                else:
                        print("Receive message '" + str(msg.payload) + "' on topic '" + msg.topic + "' with QoS " + str(msg.qos))



 

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(server, 1883, 60)
client.loop_start()
client.publish('Greeting from new Raspberry Pi',  myserial,1)

#Add_new_tsk("test","1","2","3","4","5")


#-----------------------------------------    Main loop         -----------------------------------------------
while True:
        try:
                et.write('config.xml')
                time.sleep(10)
                
                if(register_flag == False):
                        client.publish('Greeting from new Raspberry Pi',  myserial,1)
        except (IOError, TypeError) as e:
                print("Error")
