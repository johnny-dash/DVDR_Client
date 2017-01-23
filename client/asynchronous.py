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
import datetime
import grovepi
import paho.mqtt.client as mqtt
import threading
import multiprocessing
import xml.etree.ElementTree as ET

#get device serial number
from GrovepiSerial import getserial
from NetworkChecking import internet_on
from SerialDisplay import serialDisplay


#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------          Inital Configuration                     ----------------------------
#----------------------------------------------------------------------------------------------------------------------

#Process Pool
pool = []

# SIG,NC,VCC,GND
#air_sensor = 0                  # Connect the Grove Air Sensor to analog port A0
#dht_sensor_port = 8		 # Connect the DHt sensor to port 8

#Display serial number on LCD
serialDisplay()

#set up serial number
myserial = getserial()

#inital xml configuration file
global root
global tasklist
global et
try:
        #read the configration file
        et = ET.parse('config.xml')
        #set up root for device config info
        root = et.getroot()
        #set up task element for task info
        tasklist = root.find('tasklist')
        
        #the flag that if the device has been registered
        register_flag = root.find('register_flag').text
except ImportError:
        print("Import Error")
        


#MQTT Server address
server = '130.56.250.107'

#subscribe topic
sub_topic = '#' #subscribe all topics
pub_register = 'new device' #publish topic for device configuration


#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------          Sensor function                     ---------------------------------
#----------------------------------------------------------------------------------------------------------------------

def air_quality_sensor(frequence,port,tsk_id):
        while True:
                try:
                        #format port
                        air_sensor = int(port[1])
                        #grovepi configuration
                        client = mqtt.Client()
                        client.connect(server, 1883, 60)
                        r = grovepi.analogRead(air_sensor)
                        #generate unique topic for air quality sensor
                        pub_air = myserial + ':' + tsk_id + ':' + port                          
                        client.publish(pub_air, "%s@%s" % (r, datetime.datetime.now()), 1)
                        time.sleep(frequence)

                except IOError:
                        print ("Error") 

def temperature_humidity_sensor(frequence,port,tsk_id):
        while True:
                try:
                        #format port
                        dht_sensor_port = int(port[1])
                        client = mqtt.Client()
                        client.connect(server, 1883, 60)
                        #sensor config and value read
                        [ temp,hum ] = grovepi.dht(dht_sensor_port,1)
                        #setup topic for temperature&humidity sensor
                        pub_temp = myserial + ':' + tsk_id +':' + port
                        #public the message
                        client.publish(pub_temp, "%s@%s" % (temp, datetime.datetime.now()), 1)
                        time.sleep(frequence)
                
                except (IOError,TypeError) as e:
                        print("Error")

def light_sensor(frequence, port,tsk_id):
        while True:
                try:
                        #format port
                        light_sensor = int(port[1])
                        client = mqtt.Client()
                        client.connect(server, 1883, 60)
                        sensor_value = grovepi.analogRead(light_sensor)
                        #setup the topic
                        pub_light = myserial + ':' + tsk_id  + ':' + port
                        #public message
                        client.publish(pub_light,"%s@%s" % (sensor_value, datetime.datetime.now()), 1)
                        time.sleep(frequence)

                except (IOError, TypeError) as e:
                        print("Error")
                        

#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------          Process Manager                     ---------------------------------
#----------------------------------------------------------------------------------------------------------------------
def createtsk(target, tsk_freq, tsk_port, tsk_id):
        #start the corresponding function as process
        thr = multiprocessing.Process(target = target, args=(float(tsk_freq),tsk_port,tsk_id))      #create the thread to run the corresponding sensor function
        thr.start()
        #append the process into process pool
        pool.append((tsk_id,thr))

def stoptsk(tsk_id):
        #iterate to find exsisted process
        for Key, Value in pool:
                if(Key == tsk_id):
                        #kill the process and remove from pool
                        Value.terminate()
                        pool.remove((Key,Value))

def updatetsk(target, tsk_freq, tsk_port, tsk_id):
        #iterate to find exsisted process
        for Key, Value in pool:
                if(Key == name):
                        #kill the process and remove from pool
                        Value.terminate()
                        pool.remove((Key,Value))
                                
        #start the corresponding function as process
        thr = multiprocessing.Process(target = target, args=(float(tsk_freq),tsk_port,tsk_id))      #create the thread to run the corresponding sensor function
        thr.start()
        #append the process into process pool
        pool.append((tsk_id,thr))

#action filter to take different actions
def tskAction(function, action, set_sensor, set_fre, set_port, set_enroll, set_tskid ):
        if(action == 'start'):
                #create thread
                createtsk(function, set_fre, set_port, set_tskid)
                #add task into xml file
                Add_new_tsk(set_tskid, set_sensor, action, set_fre, set_port, set_enroll)
                
        elif(action == 'stop'):
                #delete the thread
                stoptsk(set_tskid)
                Change_state(set_tskid,'stop')
                
        elif(action == 'restart'):
                #restart stop task
                for task in tasklist:
                        if task.get('name') == set_tskid:
                                createtsk(function, set_fre , set_port, set_tskid)

        elif(action == 'delete'):
                #delete the thread and delete task info in xml
                for task in tasklist:
                        if task.get('name') == set_tskid:
                                if task.find('status').text != 'stop':
                                        print("hello world")
                                        stoptsk(set_tskid)
                Remove_tsk(set_tskid)
                
        elif(action == 'update'):
                #update the thread and update the xml config file
                for task in tasklist:
                        if task.get('name') == set_tskid:
                                if task.find('status').text != 'stop':
                                        updatetsk(function, set_fre, set_port, set_tskid)
                                        Remove_tsk(set_tskid)
                                        Add_new_tsk(set_tskid, set_sensor, action, set_fre, set_port, set_enroll)
                                else:
                                        Change_tsk(set_tskid, set_fre, set_port)

#filter different sensor
def whichTask(sensor, status, frequency, port, enrollment, tsk_id):
        if(sensor == 'air_quality'):
                tskAction(air_quality_sensor, status, sensor, frequency, port, enrollment, tsk_id)                                
                                                                        
        elif(sensor == 'temperature_humidity'):
                tskAction(temperature_humidity_sensor, status, sensor, frequency, port, enrollment, tsk_id)
                                                                
        elif(sensor == 'light'):
                tskAction(light_sensor, status, sensor, frequency, port, enrollment, tsk_id)
                                                               
        else:
                print('no corresponding task on client')
                
#----------------------------------------------------------------------------------------------------------------------
#---------------------------------------               XML                      ---------------------------------------
#----------------------------------------------------------------------------------------------------------------------
#this function is used to modified the xml config file by adding new task info
def Add_new_tsk(tsk_name, sensor, status, frequency, port, enrollment):
        tsk = ET.SubElement(tasklist, "task")
        tsk.set('name', tsk_name)
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
        ET.dump(tasklist)
        et.write('config.xml')

#remove the task info from xml file
def Remove_tsk(tsk_name):
        for task in tasklist:
                if task.get('name') == tsk_name:
                        tasklist.remove(task)
        et.write('config.xml')

#change the state of task
def Change_state(tsk_name, status):
        for task in tasklist:
                if task.get('name') == tsk_name:
                        task.find('status').text = status
                        et.write('config.xml')

#change parameter of task
def Change_tsk(tsk_name, frequency, port):
        for task in tasklist:
                if task.get('name') == tsk_name:
                        task.find('frequency').text = frequency
                        task.find('port').text = port
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
                        tsk_id = info[5]
                        
                        #check the output
                        print("sensor:" + sensor + " status:" + status + " frequency:" + frequency + " port:" + port + " enrollment:" + enrollment + " tsk id:" + tsk_id)
                        #check which task to do
                        whichTask(sensor, status, frequency, port, enrollment, tsk_id)

                elif topic == 'device':
                        global register_flag
                        #filter different device action
                        if str(msg.payload) == "record":
                                #the device serial number has been stored into unregistered_device db and change the flag into true
                                print("device has been detected")
                                register_flag = True
                                root.find('register_flag').text = "True"
                        elif str(msg.payload) == "unregistered":
                                #the device serial number has been deleted from registered_device db and change the flag into False
                                print("device has been unregistered")
                                register_flag = False
                                root.find('register_flag').text = "False"
                        else:
                                print('device config fail')         
                       
                else:
                        print("Receive message '" + str(msg.payload) + "' on topic '" + msg.topic + "' with QoS " + str(msg.qos))


while True:
        if internet_on():
                print('Wifi has been connected')
                break
        else:
                print('Wifi has not been connected')
        time.sleep(10)
 

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(server, 1883, 60)
client.loop_start()
client.publish('Greeting from new Raspberry Pi',  myserial,1)



#restart all the interrupted task
for task in tasklist:
        if task.find('status').text == 'start':
                whichTask(task.find('sensor').text, 'restart', task.find('frequency').text, task.find('port').text, task.find('enrollment').text, task.get('name')) 



#-----------------------------------------    Main loop         -----------------------------------------------
while True:
        try:
                et.write('config.xml')
                time.sleep(10)
                #if flag is false then keep sending greeting message every ten second 
                if(register_flag == False):
                        client.publish('Greeting from new Raspberry Pi',  myserial,1)
        except (IOError, TypeError) as e:
                print("Error")
