import sys


sys.path.insert(1, '/home/pi/Desktop/HomeMonitoring/libraries')



#import the class definition from "email_handler.py" file
from email_handler import Class_eMail

#set the email ID where you want to send the email 
To_Email_ID = "mkuliberda@gmail.com"
DETECTIONS_PATH = '/home/pi/Desktop/Detections/'


# Send Plain Text Email 
#email = Class_eMail()
#email.send_Text_Mail(To_Email_ID, 'Plain Text Mail Subject', 'This is sample plain test email body.')
#del email


# Send HTML Email
email = Class_eMail()
#email.send_HTML_Mail(To_Email_ID, 'Home Monitoring', '<html><h1>Someone is in the house</h1></html>')
fp = open(DETECTIONS_PATH + 'Detection_latest.jpg', 'rb')
email.send_Image_Mail(To_Email_ID, 'Home Monitoring', fp.read())
fp.close()
 
del email

