######## Picamera Home Monitoring Using Tensorflow Classifier and environmental sensors #########
#
# Author: Mateusz Kuliberda
# Date: 7/05/19
# Description: 
# This program uses a TensorFlow classifier to perform object detection.
# It loads the classifier uses it to perform object detection on a Picamera feed.
# It draws boxes and scores around the objects of interest in each frame from
# the Picamera.
# Additionally it connects to sensors connected to RPi, gathers measurements, saves them
# in csv file

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py


# Import packages
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys
import RPi.GPIO as GPIO
import glob
import datetime
import time
import Adafruit_DHT
import Adafruit_BMP.BMP085 as BMP085
import smbus
import time
from threading import Thread
import matplotlib.pyplot as plot
from pmsA003 import *


# Pin control
relay_pin=24
fan_pin=18
GPIO.setmode(GPIO.BCM)
GPIO.setup(relay_pin,GPIO.OUT)
GPIO.setup(fan_pin,GPIO.OUT)

# Global variables
global data_ready
data_ready = False
global environment
environment = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
global now
now = datetime.datetime.now()
global date_log
global cpu_temp

SCHEDULE_ON = 8
SCHEDULE_OFF = 22
AQI_SENS_DEV_ADDRESS = '/dev/ttyS0'


# CPU temperature reading
def get_cpu_temperature():
        temp=0
        try:
                temp = os.popen("vcgencmd measure_temp").readline()
                
        except:
                temp = cpu_temp
                print('Could not read cpu temperature!')
                
        return (temp.replace("temp=",""))

# Environment conditions reading
def get_environment_conditions():

        try:
                aqi_sensor = pmsA003(AQI_SENS_DEV_ADDRESS)
                pm = aqi_sensor.read_data()
        except:
                print('pms7003 sensor error')
        
        try:
                bus = smbus.SMBus(1)
                try:
                        bus.read_byte(0x77)
                        baro_sensor = BMP085.BMP085()
                        baro_sensor = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES)
                        pressure = baro_sensor.read_pressure()/100
                        
                except:
                        print('BMP sensor error')
        except:
                print('Wrong bus')

        
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.AM2302, 4)
        
        if humidity >= 0.0 and humidity <= 100.0 and temperature is not None and pressure is not None:
                #print('Temp: {0:0.1f} C  Humidity: {1:0.1f} %'.format(temperature, humidity))
                dew_point = (humidity/100.0) ** 0.125*(112+0.9*temperature)+0.1*temperature-112
                return [pressure, humidity, temperature, dew_point, pm[1], pm[2], pm[3]]
        else:
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                #print('Failed to get correct readings')

# Environment conditions gathering and logging thread
class gatherMeasurements(object):
        def __init__(self):
                self._running = True

        def terminate(self):
                self._running = False
                
        def run(self):
                
                global data_ready
                global environment
                global now
                global date_log
                global cpu_temp

                while self._running:

                        now = datetime.datetime.now()
                        environment_log_file = '/home/pi/Desktop/Environment/Environment_' + str(now.day) + '_' + str(now.month) + '_' + str(now.year) + '.csv'

                        if not os.path.isfile(environment_log_file):
                                f = open(environment_log_file, 'a')
                                f.write('Time,Pressure,Humidity,Temperature,Dew_point,PM1,PM2.5,PM10' + '\r\n')
                                f.close()

                        environment = get_environment_conditions()
                        cpu_temp = get_cpu_temperature()

                        if environment[0] != 0.0:
                                with open(environment_log_file, 'a') as f2:
                                        date_log = datetime.datetime.now()
                                        s2 = '{:02}'.format(int(date_log.hour)) + ':' + '{:02}'.format(int(date_log.minute)) + ':' + '{:02}'.format(int(date_log.second)) + \
                                        ',' + '{0:.2f}'.format(environment[0]) + ',' + '{0:.1f}'.format(environment[1]) + ',' + '{0:.1f}'.format(environment[2]) + ',' + \
                                        '{0:.1f}'.format(environment[3]) + ',' + '{}'.format(environment[4]) + ',' + '{}'.format(environment[5]) + ',' + '{}'.format(environment[6]) + '\r\n'
                                        f2.write(s2)
                                data_ready = True
                        time.sleep(30)


       

# Set up camera constants
IM_WIDTH = 1280
IM_HEIGHT = 720
#IM_WIDTH = 640    #Use smaller resolution for
#IM_HEIGHT = 480   #slightly faster framerate

# Select type (if user enters --noaddons when calling this script,
# it will be pure objects detection)
camera_type = 'picamera_env'
parser = argparse.ArgumentParser()
parser.add_argument('--noaddons', help='Object detection without environmental conditions and logging',
                    action='store_true')
args = parser.parse_args()
if args.noaddons:
    camera_type = 'picamera_noaddons'

# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 90

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

                

# Turn on cooling
#GPIO.output(fan_pin, GPIO.HIGH)
print('Press q to exit..')

# Initialize camera and perform object detection.
### Picamera with environment conditions and logging ###
if camera_type == 'picamera_env':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)

    #Prepare log files
    now = datetime.datetime.now()
    detections_log_file = '/home/pi/Desktop/Detections/Detection_' + str(now.day) + '_' + str(now.month) + '_' + str(now.year) + '.csv'

    #Start separate thread for environment measurements
    #Create class
    measurements = gatherMeasurements()
    #Create Thread
    gatherMeasurementsThread = Thread(target = measurements.run)
    #Start Thread
    gatherMeasurementsThread.start()
    
    

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_expanded = np.expand_dims(frame, axis=0)

        now = datetime.datetime.now()
        if int(now.hour) >= SCHEDULE_ON and int(now.hour) < SCHEDULE_OFF:

                GPIO.output(fan_pin, GPIO.HIGH)
                t1 = cv2.getTickCount()
                
                # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
                # i.e. a single-column array, where each item in the column has the pixel RGB value
                #frame = np.copy(frame1.array)
                #frame.setflags(write=1)
                #frame_expanded = np.expand_dims(frame, axis=0)

                # Perform the actual detection by running the model with the image as input
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: frame_expanded})

                # Draw the results of the detection (aka 'visualize the results')
                vis_util.visualize_boxes_and_labels_on_image_array(
                    frame,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=8,
                    min_score_thresh=0.7)

                
                if(data_ready == True):
                        environment_valid = environment
                        data_ready = False
                        
                date_log = str(datetime.datetime.now())

                overlay = frame.copy()
                alpha = 0.7 # Transparency factor
                
                cv2.putText(overlay,"FPS: {0:.2f} ".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"CPU: " + cpu_temp,(200,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"Environment:",(30,120),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"Pressure: {0:.2f} hPa".format(environment_valid[0]),(30,150),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"Humidity: {0:.1f} %".format(environment_valid[1]),(30,180),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"Temperature: {0:.1f} 'C".format(environment_valid[2]),(30,210),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"Dew Point: {0:.1f} 'C".format(environment_valid[3]),(30,240),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"PM1: {} ug/m3".format(environment_valid[4]),(30,270),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"PM2.5: {} ug/m3".format(environment_valid[5]),(30,300),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(overlay,"PM10: {} ug/m3".format(environment_valid[6]),(30,330),font,1,(255,255,0),2,cv2.LINE_AA)
               
                
                # Class 1 represents human
                if ((classes[0][0] == 1 and scores[0][0] > 0.75) or (classes[0][1] == 1 and scores[0][1] > 0.75) or (classes[0][2] == 1 and scores[0][2] > 0.75)):
                    cv2.putText(overlay,"Human detected!",(30,80),font,1,(255,255,0),2,cv2.LINE_AA)
                    cv2.imwrite("/home/pi/Desktop/Detections/Detection_Frame_%s.jpg" % date_log, frame)
                    #GPIO.output(relay_pin,True)
                else:
                    cv2.putText(overlay,"No Human detected!",(30,80),font,1,(255,255,0),2,cv2.LINE_AA)
                    #GPIO.output(relay_pin,False)
                    
                if ((classes[0][0] == 17 and scores[0][0] > 0.75) or (classes[0][1] == 17 and scores[0][1] > 0.75) or (classes[0][2] == 17 and scores[0][2] > 0.75)):
                    cv2.putText(overlay,"Cat detected!",(380,80),font,1,(255,255,0),2,cv2.LINE_AA)
                    cv2.imwrite("/home/pi/Desktop/Detections/Detection_Frame_%s.jpg" % date_log, frame)

                image = cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0)
                # All the results have been drawn on the frame, so it's time to display it.
                cv2.imshow('Object detector', image)

                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc = 1/time1

        else:
                cv2.putText(frame,"Object detector is OFF",(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.imshow('Object detector', frame)
                GPIO.output(fan_pin, GPIO.LOW)


        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)

        
    GPIO.output(fan_pin, GPIO.LOW)
    GPIO.cleanup()
    measurements.terminate()
    camera.close()

if camera_type == 'picamera_noaddons':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_expanded = np.expand_dims(frame, axis=0)

        now = datetime.datetime.now()
        if int(now.hour) >= SCHEDULE_ON and int(now.hour) < SCHEDULE_OFF:

                GPIO.output(fan_pin, GPIO.HIGH)
                t1 = cv2.getTickCount()
                
                # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
                # i.e. a single-column array, where each item in the column has the pixel RGB value
                #frame = np.copy(frame1.array)
                #frame.setflags(write=1)
                #frame_expanded = np.expand_dims(frame, axis=0)

                # Perform the actual detection by running the model with the image as input
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: frame_expanded})

                # Draw the results of the detection (aka 'visualize the results')
                vis_util.visualize_boxes_and_labels_on_image_array(
                    frame,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=8,
                    min_score_thresh=0.1)

                cpu_temp = get_cpu_temperature()

                cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.putText(frame,"CPU: " + cpu_temp,(200,50),font,1,(255,255,0),2,cv2.LINE_AA)
                
                cv2.imshow('Object detector', frame)

                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc = 1/time1

        else:
                cv2.putText(frame,"Object detector is OFF",(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.imshow('Object detector is OFF', frame)
                GPIO.output(fan_pin, GPIO.LOW)
        

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)

        
    GPIO.output(fan_pin, GPIO.LOW)
    GPIO.cleanup()
    camera.close()
    


cv2.destroyAllWindows()

