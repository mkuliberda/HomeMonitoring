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
from threading import Event, Thread
import threading
from pmsA003 import *
import tty
import termios
import io
import http.server
import socketserver
from subprocess import Popen

#TODO: move all the configs to JSON

# Server port and address
PORT=8081

# Pin control
FAN=18

# Refresh rates
DISARMED_REFRESH_RATE_S = 10

# Global variables
validated_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', 0.0, 0.0, 0.0]
detector_ctrl = {'mode' : 'SCHEDULED', 'index' : 0, 'armed': False}
validated_gradients = {'Pressure' : 0.0, 'Humidity' : 0.0, 'Temperature' : 0.0}

SCHEDULE_ON = 9
SCHEDULE_OFF = 16
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
lock = threading.RLock()
plottingEvent = threading.Event()


# Environment conditions gathering and logging thread
class measurementsThread(threading.Thread):
        def __init__(self):

                self._running = True
                self.lat = 54.390819
                self.lon = 18.599229
                self.alt = 50.0
                self._data_ready = False
                self.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', self.lat, self.lon, self.alt]
                self.gradients = {'Pressure' : 0.0, 'Humidity' : 0.0, 'Temperature' : 0.0}
                self.pressureBuffer = [0.0] * 120 # 60min buffer
                self.humidityBuffer = [0.0] * 120 # 60min buffer
                self.temperatureBuffer = [0.0] * 120 # 60min buffer 
                threading.Thread.__init__(self)

        def terminate(self):
                self._running = False

        def setConsumed(self):
                self._data_ready = False

        def setProduced(self):
                self._data_ready = True
        
        def checkDataAvbl(self):
                return self._data_ready
        
        def getAllData(self):
                return self.data

        def checkRunning(self):
                return self._running

        def calculateDewPoint(self, env_hum, env_temp):
                return (env_hum/100.0) ** 0.125*(112+0.9*env_temp)+0.1*env_temp-112

        def updateBuffer(self, current_buffer, new_sample):

                size = len(current_buffer)
                new_buffer = np.roll(current_buffer,-1)
                new_buffer[size-1] = new_sample
                
                return new_buffer

        def calculateGradient(self, buffer):

                size = len(buffer)
                x = np.linspace(0, len(buffer)-1, size)
                gradient_coeffs = np.polyfit(x,buffer,1)
                extremum = [gradient_coeffs[1] , (size-1)*gradient_coeffs[0]+gradient_coeffs[1]]
                grad = (extremum[1] - extremum[0])
                
                return grad

        def getGradients(self):
                return self.gradients

        def saveMeasurements(self, measurements):

                now = datetime.datetime.now()
                environment_log_file = ENVIRONMENT_PATH + '/Environment_' + str(now.day) + '_' + str(now.month) + '_' + str(now.year) + '.csv'
                
                if not os.path.isfile(environment_log_file):
                        with open(environment_log_file, 'a') as f:
                                f.write('Time,Pressure,Humidity,Temperature,Dew_point,PM1,PM2.5,PM10,Lat,Lon,Alt' + '\r\n')

                with open(environment_log_file, 'a') as f:
                        s = '{:02}'.format(int(now.hour)) + ':' + '{:02}'.format(int(now.minute)) + ':' + '{:02}'.format(int(now.second)) + \
                        ',' + '{0:.2f}'.format(measurements[0]) + ',' + '{0:.1f}'.format(measurements[1]) + ',' + '{0:.1f}'.format(measurements[2]) + ',' + \
                        '{0:.1f}'.format(measurements[3]) + ',' + '{}'.format(measurements[4]) + ',' + '{}'.format(measurements[5]) + ',' + '{}'.format(measurements[6]) + \
                        ',' + '{}'.format(measurements[8]) + ',' + '{}'.format(measurements[9]) + ',' + '{}'.format(measurements[10]) + '\r\n'
                        f.write(s)

        def getTemperatureCPU(self):
                try:
                        f = open("/sys/class/thermal/thermal_zone0/temp", "r")
                        t = float(f.readline ())
                        temp = "temp: {0:.1f}C".format(t/1000)
                except:
                        temp = '0'
                        print('Could not read cpu temperature!')
                        
                return temp
        
        def getEnvironmentalConditions(self):
                
                pm = [None, None, None, None]
                pressure = None
                humidity = None
                temperature = None
                dew_point = None  
 
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
                        print('BMP085 wrong bus')

                try:
                        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.AM2302, 4)
                except:
                        print('DHT sensor error')
                        
                if humidity is not None and temperature is not None and pressure is not None and pm[1] is not None and pm[2] is not None and pm[3] is not None:
                        if humidity >= 0.0 and humidity <= 100.0 and temperature >-40.0 and temperature < 80.0:
                                temperature = temperature - 1.0 #account for sensor and rpi self heating by approx 1C TODO: this needs to be improved by reduction for detector work
                                dew_point = self.calculateDewPoint(humidity,temperature)
                                return [pressure, humidity, temperature, dew_point, pm[1], pm[2], pm[3], '0', self.lat, self.lon, self.alt]
                        else:
                                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', self.lat, self.lon, self.alt]
                else:
                        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '0', self.lat, self.lon, self.alt]

        def run(self):

                while self._running:

                        self.data = self.getEnvironmentalConditions()
                        self.data[7] = self.getTemperatureCPU()

                        self.pressureBuffer = self.updateBuffer(self.pressureBuffer[:], self.data[0])
                        self.gradients['Pressure'] = self.calculateGradient(self.pressureBuffer)

                        self.humidityBuffer = self.updateBuffer(self.humidityBuffer[:], self.data[1])
                        self.gradients['Humidity'] = self.calculateGradient(self.humidityBuffer)

                        self.temperatureBuffer = self.updateBuffer(self.temperatureBuffer[:], self.data[2])
                        self.gradients['Temperature'] = self.calculateGradient(self.temperatureBuffer)


                        if self.data[0] != 0.0:
                                self.saveMeasurements(self.data[:])

                        self.setProduced()
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



class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Handle requests in a separate thread."""


class MyRequestHandler(http.server.SimpleHTTPRequestHandler):

        def __init__(self, request, client_address, server):

                self.data = server.data
                self.det_ctrl = server.det_ctrl
                self.gradients = server.gradients
                http.server.SimpleHTTPRequestHandler.__init__(self, request, client_address, server)

                #self.data = data_list
                #os.chdir(OBJECTDETECTION_PATH)
                #with open("favicon.ico", "rb") as favicon:
                #        self.favicon = favicon.read()
            
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

                      
                html = '''
                <html>
                <body style="width:500px; margin:auto">
                <h1><center>HOME MONITOR</center></h1>
                <h2><center>Current status and environment</center></h2>
                <table border=1 width="500">
                <tr>
                <td style="text-align:center">Pressure (dP) [hPa (hPa/h)]</th>
                <td style="text-align:center">{} ({})</th>
                </tr>
                <tr>
                <td style="text-align:center">Humidity (dH) [% (%/h)]</th>
                <td style="text-align:center">{} ({})</th>
                </tr>
                <tr>
                <td style="text-align:center">Air Temperature (dT) [C (C/h)]</th>
                <td style="text-align:center">{} ({})</th>
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

                </br>
                <h2><center>User control</center></h2>
                <center><table width="500">
                <tr>
                <td style="text-align:center">
                <form action="/" method="POST">
                        <input type="submit" name="submit" value="Refresh" style="height:40px; width:170px">
                </form>
                </th>
                <td style="text-align:center">
                <form action="/" method="POST">
                        <input type="submit" name="submit" value="ToggleDetectorMode" style="height:40px; width:170px">
                </form>
                </th>
                </tr>
                </table></center>

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
                self.wfile.write(html.format('{0:.2f}'.format(self.data[0]),'{0:.3f}'.format(self.gradients['Pressure']), \
                                                '{0:.1f}'.format(self.data[1]),'{0:.2f}'.format(self.gradients['Humidity']), \
                                                '{0:.1f}'.format(self.data[2]),'{0:.2f}'.format(self.gradients['Temperature']), \
                                                '{0:.1f}'.format(self.data[3]),self.data[4],self.data[5],self.data[6],self.data[7],self.det_ctrl['mode']).encode("utf-8"))     

 
        def do_POST(self):
                content_length = int(self.headers['Content-Length'])    # Get the size of data
                post_data = self.rfile.read(content_length).decode("utf-8")   # Get the data
                post_data = post_data.split("=")[1]    # Only keep the value
                

                if post_data == 'Refresh':
                        self._redirect('/')    # Redirect back to the root url
                        
                elif post_data == 'ToggleDetectorMode':
                        DETECTOR_MODE_OPT = ('SCHEDULED','FORCE_OFF','FORCE_ON')
                        self.det_ctrl['index'] += 1
                        if self.det_ctrl['index'] >= len(DETECTOR_MODE_OPT):
                                self.det_ctrl['index'] = 0
                        self.det_ctrl['mode'] = DETECTOR_MODE_OPT[self.det_ctrl['index']]
                        self._redirect('/')    # Redirect back to the root url

        def log_message(self, format, *args):
                return

class notificationsThread(threading.Thread):
        def __init__(self,mode = {'email': False, 'sms' : False}):
                self._send = False
                self._running = True
                self._mode = mode
                
                threading.Thread.__init__(self)

        def send_notification(self):
                self._send = True

        def run(self):
                while self._running == True:
                        if self._send == True:

                                if self._mode.get('email') == True:
                                        os.chdir(REPOSITORY_PATH)
                                        os.system('python notify.py')
                                        os.chdir(OBJECTDETECTION_PATH)

                                if self._mode.get('sms') == True:
                                        print('sms send not implemented yet')

                                self._send = False
                        time.sleep(10)
                
        def terminate(self):
                self._running = False

def createPlots(event):

        while True:

                plottingEvent.wait()
        
                try:
                        os.chdir(STATISTICS_PATH)
                        os.system('python3 logs_statistics.py --image')
                        os.chdir(OBJECTDETECTION_PATH)
                        plottingEvent.clear()
                except:
                        plottingEvent.clear()
                        break


 
                              


#create breaker thread for exit from camera loop
print('Detector active: Monday - Friday, ' + '{:02}'.format(SCHEDULE_ON) + ':00 - ' + '{:02}'.format(SCHEDULE_OFF) + ':00')
print('Press q to exit. Press e to print all measurements')

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
    
    #Start threaded web server    
    webpage = ThreadedHTTPServer(('0.0.0.0', PORT), MyRequestHandler)
    with lock:
            webpage.data = validated_data
            webpage.gradients = validated_gradients
            webpage.det_ctrl = detector_ctrl
    print('Starting server...')
    
    serverThread = threading.Thread(target = webpage.serve_forever)
    serverThread.deamon = False
    serverThread.start()
    print("Serving webpage at port:",PORT) 

    #Start separate thread for email notifications
    notifications = notificationsThread({'email': True, 'sms' : False})
    notifications.start()
    
    #Start separate thread for environment measurements
    collector = measurementsThread()
    collector.start()
    
    #Start separate thread for plotting of measurements
    plotsThread = threading.Thread(target=createPlots, args=(plottingEvent,))
    plotsThread.daemon = True
    plotsThread.start()

        

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        now = datetime.datetime.now()
        if(collector.checkDataAvbl() == True):
                with lock:
                        validated_data = collector.getAllData()
                        validated_gradients = collector.getGradients()
                        webpage.data = validated_data
                        webpage.gradients = validated_gradients
                        detector_ctrl = webpage.det_ctrl
                plottingEvent.set()
                collector.setConsumed()

        if (int(now.hour) >= SCHEDULE_ON and int(now.hour) < SCHEDULE_OFF and now.weekday() != 5 and now.weekday() != 6 and detector_ctrl['mode'] == 'SCHEDULED') or detector_ctrl['mode'] == 'FORCE_ON':

                with lock:
                        detector_ctrl['armed'] = True

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


                overlay = frame.copy()
                alpha = 0.7 # Transparency factor
                
                #cv2.putText(overlay,"FPS: {0:.2f} ".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"CPU " + validated_data[7],(200,50),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Environment:",(30,120),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Pressure: {0:.2f} hPa".format(validated_data[0]),(30,150),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Humidity: {0:.1f} %".format(validated_data[1]),(30,180),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Temperature: {0:.1f} 'C".format(validated_data[2]),(30,210),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"Dew Point: {0:.1f} 'C".format(validated_data[3]),(30,240),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"PM1: {} ug/m3".format(validated_data[4]),(30,270),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"PM2.5: {} ug/m3".format(validated_data[5]),(30,300),font,1,(255,255,0),2,cv2.LINE_AA)
                #cv2.putText(overlay,"PM10: {} ug/m3".format(validated_data[6]),(30,330),font,1,(255,255,0),2,cv2.LINE_AA)
                    
                now = str(datetime.datetime.now())
                
                # Class 1 represents human
                if ((classes[0][0] == 1 and scores[0][0] > 0.75) or (classes[0][1] == 1 and scores[0][1] > 0.75) or (classes[0][2] == 1 and scores[0][2] > 0.75)):
                        cv2.putText(overlay,"Human detected! on " + now,(30,80),font,1,(255,255,0),2,cv2.LINE_AA)
                        cv2.imwrite(DETECTIONS_PATH + '/Detection_Frame_%s.jpg' % now, frame)
                        cv2.imwrite(DETECTIONS_PATH + '/Detection_latest.jpg', overlay)
                        notifications.send_notification()

                else:
                        cv2.putText(overlay,"Detector ARMED. No Human detected!",(30,80),font,1,(255,255,0),2,cv2.LINE_AA)
                    
                image = cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0)
                # All the results have been drawn on the frame, so it's time to display it.
                cv2.imwrite(DETECTIONS_PATH + '/Detector.jpg', image)
                #cv2.imshow('Object detector', image)

                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc = 1/time1

        else:
                frame = np.copy(frame1.array)
                frame.setflags(write=1)
                frame_expanded = np.expand_dims(frame, axis=0)
                cv2.putText(frame,"Detector DISARMED. Refreshing picture every {}s".format(DISARMED_REFRESH_RATE_S),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.imwrite(DETECTIONS_PATH + '/Detector.jpg', frame)
                with lock:
                        detector_ctrl['armed'] = False
                #cv2.imshow('Object detector', frame)
                        
                cooler.turnOFF()
                time.sleep(DISARMED_REFRESH_RATE_S)

        if breakNow == True:
                print('closing, please wait few seconds for terminal..')
                break

        if envPrint == True:
                print('Pressure, Humidity, Temperature, Dew_point, PM1, PM2.5, PM10, CPU, Latitude, Longitude, Altitude, Gradients')
                print(validated_data, validated_gradients)
                envPrint = False

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
                print('press q in terminal to exit')

        rawCapture.truncate(0)

    cooler.turnOFF()
    cooler.cleanSys()
    print('cooler off')
    
    collector.terminate()
    collector.join(35)
    del collector
    print('collector off')

    notifications.terminate()
    notifications.join(15)
    del notifications
    print('notifications off')

    webpage.shutdown()
    del webpage
    print('server off')

    camera.close()
    print('camera off')


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
                cv2.putText(frame,"Detector DISARMED. Refreshing picture every {}s".format(DISARMED_REFRESH_RATE_S),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
                cv2.imshow('Object detector', frame)
                cooler.turnOFF()
                time.sleep(DISARMED_REFRESH_RATE_S)
                

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
print('exiting...')
cv2.destroyAllWindows()

