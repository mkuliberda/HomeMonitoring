######## Picamera Home Monitoring Using Tensorflow Classifier and environmental sensors #########
#
# Author: Mateusz Kuliberda
# Date: 12/05/19
# Description: 
# This program uses a TensorFlow classifier to perform object detection.
# It loads the classifier uses it to perform object detection on a Picamera feed.
# It draws boxes and scores around the objects of interest in each frame from
# the Picamera.
# Additionally it connects to sensors connected to RPi, gathers measurements, saves them
# in csv file and serves a webpage to present the data

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
# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util
import RPi.GPIO as GPIO
import glob
import datetime
import time
import Adafruit_DHT
import Adafruit_BMP.BMP085 as BMP085
import smbus
import time
from threading import Thread
import threading
import matplotlib.pyplot as plot
from pmsA003 import *
import tty
import termios
import io
from http.server import BaseHTTPRequestHandler, HTTPServer
from subprocess import Popen



# Server port and address
PORT=8080

# Pin control
FAN=18

# Global variables
data_ready = False
environment = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', 0.0, 0.0, 0.0]
environment_valid = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', 0.0, 0.0, 0.0]
now = datetime.datetime.now()
date_log = 0
detector_mode = 'SCHEDULED'
detector_ctrl_index = 0

SCHEDULE_ON = 9
SCHEDULE_OFF = 16
DETECTOR_MODE_OPT = ('SCHEDULED','FORCE_OFF','FORCE_ON')
AQI_SENS_DEV_ADDRESS = '/dev/ttyS0'

#Paths
OBJECTDETECTION_PATH = '/home/pi/tensorflow1/models/research/object_detection'
STATISTICS_PATH = '/home/pi/Desktop/Statistics'
ENVIRONMENT_PATH = '/home/pi/Desktop/Environment'
DETECTIONS_PATH = '/home/pi/Desktop/Detections'
REPOSITORY_PATH = '/home/pi/Desktop/HomeMonitoring'

# Set up camera constants
IM_WIDTH = 1280
IM_HEIGHT = 720
#IM_WIDTH = 640    #Use smaller resolution for
#IM_HEIGHT = 480   #slightly faster framerate

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
lock = threading.Lock()



# Environment conditions gathering and logging thread
class measurements(object):
        def __init__(self):
                self._running = True
                self.temp = ""
                self.lat = 54.390819
                self.lon = 18.599229
                self.alt = 50.0


        def terminate(self):
                self._running = False

        # CPU temperature reading
        def get_cpu_temperature(self):
                try:
                        f = open("/sys/class/thermal/thermal_zone0/temp", "r")
                        t = float(f.readline ())
                        temp = "temp: {0:.1f}C".format(t/1000)
                except:
                        temp = '0'
                        print('Could not read cpu temperature!')
                        
                return temp
        
        # Environment conditions reading
        def get_environment_conditions(self):

                try:
                        aqi_sensor = pmsA003(AQI_SENS_DEV_ADDRESS)
                        pm = aqi_sensor.read_data()
                except:
                        pm = [0, 0, 0]
                        print('pms7003 sensor error')
                
                try:
                        bus = smbus.SMBus(1)
                        try:
                                bus.read_byte(0x77)
                                baro_sensor = BMP085.BMP085()
                                baro_sensor = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES)
                                pressure = baro_sensor.read_pressure()/100
                                
                        except:
                                pressure = None
                                print('BMP sensor error')
                except:
                        print('Wrong bus')

                try:
                        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.AM2302, 4)
                except:
                        humidity = None
                        temperature = None
                        print('DHT sensor error')
                        
                
                if humidity is not None and temperature is not None and pressure is not None:
                        if humidity >= 0.0 and humidity <= 100.0 and temperature >-40.0 and temperature < 80.0:
                                dew_point = (humidity/100.0) ** 0.125*(112+0.9*temperature)+0.1*temperature-112
                                return [pressure, humidity, temperature, dew_point, pm[1], pm[2], pm[3], '0', self.lat, self.lon, self.alt]
                        else:
                                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', self.lat, self.lon, self.alt]
                else:
                        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', self.lat, self.lon, self.alt]

        def run(self):
                
                global data_ready
                global environment
                global now
                global date_log

                while self._running:

                        now = datetime.datetime.now()
                        environment_log_file = ENVIRONMENT_PATH + '/Environment_' + str(now.day) + '_' + str(now.month) + '_' + str(now.year) + '.csv'
                        
                        if not os.path.isfile(environment_log_file):
                                f = open(environment_log_file, 'a')
                                f.write('Time,Pressure,Humidity,Temperature,Dew_point,PM1,PM2.5,PM10,Lat,Lon,Alt' + '\r\n')
                                f.close()

                        environment = self.get_environment_conditions()
                        environment[7] = self.get_cpu_temperature()

                        if environment[0] != 0.0:
                                with open(environment_log_file, 'a') as f2:
                                        date_log = datetime.datetime.now()
                                        s2 = '{:02}'.format(int(date_log.hour)) + ':' + '{:02}'.format(int(date_log.minute)) + ':' + '{:02}'.format(int(date_log.second)) + \
                                        ',' + '{0:.2f}'.format(environment[0]) + ',' + '{0:.1f}'.format(environment[1]) + ',' + '{0:.1f}'.format(environment[2]) + ',' + \
                                        '{0:.1f}'.format(environment[3]) + ',' + '{}'.format(environment[4]) + ',' + '{}'.format(environment[5]) + ',' + '{}'.format(environment[6]) + \
                                        ',' + '{}'.format(environment[8]) + ',' + '{}'.format(environment[9]) + ',' + '{}'.format(environment[10]) + '\r\n'
                                        f2.write(s2)
                        os.chdir(STATISTICS_PATH)
                        os.system('python3 logs_statistics.py --image')
                        os.chdir(OBJECTDETECTION_PATH)
                        data_ready = True
                        time.sleep(30)

class cooling(object):
    def __init__(self,fan_pin):
        self._is_running = False
        self.pin = fan_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin,GPIO.OUT)
        
    def turnON(self):
        GPIO.output(self.pin, GPIO.HIGH)
        self._is_running = True
        
    def turnOFF(self):
        GPIO.output(self.pin, GPIO.LOW)
        self._is_running = False
        
    def getStatus(self):
        return self._is_running
    
    def cleanSys(self):
        GPIO.cleanup()

class httpserver(BaseHTTPRequestHandler):

        def __init__(self, request, client_address, server):
                
                self.html = ["Error loading main.html"]
                self.detector_ctrl = ''
                #os.chdir(OBJECTDETECTION_PATH)
                #with open("favicon.ico", "rb") as favicon:
                #        self.favicon = favicon.read()

                BaseHTTPRequestHandler.__init__(self, request, client_address, server)

            
        def do_HEAD(self):
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()

        def _redirect(self, path):
                self.send_response(303)
                self.send_header('Content-type', 'text/html')
                self.send_header('Location', path)
                self.end_headers()

        def do_GET(self):
                #print(self.path)
                #if '/favicon.ico' in self.path:
                #        self.send_response(200)
                #        self.send_header("Content-type", 'image/x-icon')
                #        self.end_headers()
                #        self.wfile.write(self.favicon)
                #        return

                if '/plots.jpg' in self.path:
                        self.send_response(200)
                        self.send_header("Content-type", 'image/jpg')
                        self.end_headers()
                        with open(ENVIRONMENT_PATH + "/plots.jpg", "rb") as image_file:
                                self.wfile.write(image_file.read())
                        return
                if '/Detector.jpg' in self.path:
                        self.send_response(200)
                        self.send_header("Content-type", 'image/jpg')
                        self.end_headers()
                        with open(DETECTIONS_PATH + "/Detector.jpg", "rb") as image_file:
                                self.wfile.write(image_file.read())
                        return

                if '/Detection_latest.jpg' in self.path:
                        self.send_response(200)
                        self.send_header("Content-type", 'image/jpg')
                        self.end_headers()
                        with open(DETECTIONS_PATH + "/Detection_latest.jpg", "rb") as image_file:
                                self.wfile.write(image_file.read())
                        return



                
                global environment_valid, detector_mode, lock #very ugly solution with those globals, TODO:change this!!!

                with lock:
                        
                        self.detector_ctrl = detector_mode

                        html = '''
                        <html>
                        <body style="width:350px; margin: 10px auto;">
                        <h1><center>HOME MONITOR</center></h1>
                        <h2><center>Current conditions</center></h2>
                        <table border=1 width="500">
                        <tr>
                        <td style="text-align:center">Pressure [hPa]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">Humidity [%]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">Air Temperature [C]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">Dew Point [C]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">PM1 [ug/m3]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">PM2.5 [ug/m3]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">PM10 [ug/m3]</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">CPU </th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        <tr>
                        <td style="text-align:center">Detector Mode</th>
                        <td style="text-align:center">{}</th>
                        </tr>
                        </table>
                        <center>
                        <form action="/" method="POST">
                                <input type="submit" name="submit" value="Refresh">
                        <form action="/" method="POST">
                                <input type="submit" name="submit" value="ToggleDetectorMode">
                        </form>
                        </center>
                        <img src="plots.jpg" alt="Plots" width="500" height="500"/>
                        </br>
                        <center><h2>Detector preview</h2></center>
                        </br>
                        <img src="Detector.jpg" alt="Detector" width="500" height="360"/>
                        </br>
                        <center><h2>Last detection</h2></center>
                        </br>
                        <img src="Detection_latest.jpg" alt="LatestDetection" width="500" height="360"/>
                        </body>
                        </html>
                        '''

                        self.do_HEAD()
                        self.wfile.write(html.format('{0:.2f}'.format(environment_valid[0]),'{0:.1f}'.format(environment_valid[1]),'{0:.1f}'.format(environment_valid[2]),'{0:.1f}'.format(environment_valid[3]),
                                                     environment_valid[4],environment_valid[5],environment_valid[6],environment_valid[7],self.detector_ctrl).encode("utf-8"))     


 
        def do_POST(self):
                content_length = int(self.headers['Content-Length'])    # Get the size of data
                post_data = self.rfile.read(content_length).decode("utf-8")   # Get the data
                post_data = post_data.split("=")[1]    # Only keep the value
                global detector_mode
                global detector_ctrl_index  #TODO: this needs to be changed to local variable
                global DETECTOR_MODE_OPT

                if post_data == 'Refresh':
                        self._redirect('/')    # Redirect back to the root url
                        
                elif post_data == 'ToggleDetectorMode':
                        with lock:
                                detector_ctrl_index += 1
                                if detector_ctrl_index >= len(DETECTOR_MODE_OPT):
                                        detector_ctrl_index = 0
                                detector_mode = DETECTOR_MODE_OPT[detector_ctrl_index]
                        self._redirect('/')    # Redirect back to the root url


        
        
                              


#create breaker thread for exit from camera loop
print('Detector active: Monday - Friday, ' + '{:02}'.format(SCHEDULE_ON) + ':00 - ' + '{:02}'.format(SCHEDULE_OFF) + ':00')
print('Press q to exit. Press e to print current environmental conditions')

breakNow = False
envPrint = False

def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return ch

def waitForKeyPress():

    global breakNow
    global envPrint

    while True:
        ch = getch()

        if ch == 'q':
                breakNow = True
                break
        if ch == 'e':
                envPrint = True



        
#Create breakerThread
breakerThread = Thread(target = waitForKeyPress)
#Start Thread
breakerThread.start()


#Instantiate cooler object for fan control
cooler = cooling(FAN)            

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

    #Start separate thread for environment measurements
    #Create class
    collector = measurements()
    #Create Thread
    measurementsThread = Thread(target = collector.run)
    measurementsThread.daemon = False
    #Start Thread
    measurementsThread.start()

    #Start separate thread for environment server
    #Create class
    webpage = HTTPServer(('', PORT), httpserver)
    #Create Thread
    serverThread = Thread(target = webpage.serve_forever)
    serverThread.daemon = True
    #Start Thread
    serverThread.start()
    print("Serving at port:",PORT)                         
    

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        with lock:    
                if(data_ready == True):
                        environment_valid = environment
                        data_ready = False

        now = datetime.datetime.now()
        if (int(now.hour) >= SCHEDULE_ON and int(now.hour) < SCHEDULE_OFF and now.weekday() != 5 and now.weekday() != 6 and detector_mode == 'SCHEDULED') or detector_mode == 'FORCE_ON':

                cooler.turnON()
                t1 = cv2.getTickCount()
                
                # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
                # i.e. a single-column array, where each item in the column has the pixel RGB value
                frame = np.copy(frame1.array)
                frame.setflags(write=1)
                frame_expanded = np.expand_dims(frame, axis=0)

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
                        
                date_log = str(datetime.datetime.now())

                overlay = frame.copy()
                alpha = 0.7 # Transparency factor
                
                #cv2.putText(overlay,"FPS: {0:.2f} ".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"CPU " + environment_valid[7],(200,50),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Environment:",(30,120),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Pressure: {0:.2f} hPa".format(environment_valid[0]),(30,150),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Humidity: {0:.1f} %".format(environment_valid[1]),(30,180),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Temperature: {0:.1f} 'C".format(environment_valid[2]),(30,210),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Dew Point: {0:.1f} 'C".format(environment_valid[3]),(30,240),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"PM1: {} ug/m3".format(environment_valid[4]),(30,270),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"PM2.5: {} ug/m3".format(environment_valid[5]),(30,300),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"PM10: {} ug/m3".format(environment_valid[6]),(30,330),font,1,(255,255,0),2,cv2.LINE_AA)
               

                # Class 1 represents human
                if ((classes[0][0] == 1 and scores[0][0] > 0.75) or (classes[0][1] == 1 and scores[0][1] > 0.75) or (classes[0][2] == 1 and scores[0][2] > 0.75)):
                        cv2.putText(overlay,"Human detected! on " + date_log,(30,80),font,1,(255,255,0),2,cv2.LINE_AA)
                        with lock:
                                cv2.imwrite(DETECTIONS_PATH + '/Detection_Frame_%s.jpg' % date_log, frame)
                                cv2.imwrite(DETECTIONS_PATH + '/Detection_latest.jpg', overlay)
                                os.chdir(REPOSITORY_PATH)
                                os.system('python notify.py')
                                os.chdir(OBJECTDETECTION_PATH)

                else:
                        cv2.putText(overlay,"No Human detected!",(30,80),font,1,(255,255,0),2,cv2.LINE_AA)
                    
                image = cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0)
                # All the results have been drawn on the frame, so it's time to display it.
                with lock:
                        cv2.imwrite(DETECTIONS_PATH + '/Detector.jpg', image)
                        #cv2.imshow('Object detector', image)

                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc = 1/time1

        else:
                frame = np.copy(frame1.array)
                frame.setflags(write=1)
                frame_expanded = np.expand_dims(frame, axis=0)
                cv2.putText(frame,"Object detector is OFF, just refreshing picture every 15s",(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                with lock:
                        cv2.imwrite(DETECTIONS_PATH + '/Detector.jpg', frame)
                        #cv2.imshow('Object detector', frame)
                        
                cooler.turnOFF()
                time.sleep(15)

        if breakNow == True:
                print('closing, please wait few seconds for terminal..')
                break

        if envPrint == True:
                print('Pressure, Humidity, Temperature, Dew_point, PM1, PM2.5, PM10, CPU, Latitude, Longitude, Altitude')
                with lock:
                        print(environment_valid)
                envPrint = False

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
                print('press q in terminal to exit')

        rawCapture.truncate(0)

    cooler.turnOFF()
    cooler.cleanSys()
    
    collector.terminate()
    measurementsThread.join()
    del collector
    del webpage
    
    
    camera.close()


if camera_type == 'picamera_noaddons':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        now = datetime.datetime.now()
        if int(now.hour) >= SCHEDULE_ON and int(now.hour) < SCHEDULE_OFF and now.weekday() != 5 and now.weekday() != 6:

                cooler.turnON()
                t1 = cv2.getTickCount()
                
                # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
                # i.e. a single-column array, where each item in the column has the pixel RGB value
                frame = np.copy(frame1.array)
                frame.setflags(write=1)
                frame_expanded = np.expand_dims(frame, axis=0)

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

                #cpu_temp = get_cpu_temperature()

                cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(frame,"CPU: " + cpu_temp,(200,50),font,1,(255,255,0),2,cv2.LINE_AA)
                
                cv2.imshow('Object detector', frame)

                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc = 1/time1

        else:
                frame = np.copy(frame1.array)
                frame.setflags(write=1)
                frame_expanded = np.expand_dims(frame, axis=0)
                cv2.putText(frame,"Object detector is OFF, refreshing picture every 15s",(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.imshow('Object detector', frame)
                cooler.turnOFF()
                time.sleep(15)
                

        if breakNow == True:
                print('closing, please wait few seconds for terminal..')
                break
        
        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
                print('press q in terminal to exit')

        rawCapture.truncate(0)

        
    cooler.turnOFF()
    cooler.cleanSys()
    camera.close()
    

del cooler
cv2.destroyAllWindows()

