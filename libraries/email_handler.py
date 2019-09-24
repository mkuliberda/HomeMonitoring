
##------------------------------------------
##--- Author: Pradeep Singh
##--- Blog: https://iotbytes.wordpress.com/programmatically-send-e-mail-from-raspberry-pi-using-python-and-gmail/
##--- Date: 21st Feb 2017
##--- Version: 1.0
##--- Python Ver: 2.7
##--- Description: This python code will send Plain Text and HTML based emails using Gmail SMTP server
##------------------------------------------


import ConfigParser, inspect, os
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage


#Form the absolute path for the settings.ini file
settings_Dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) 
settings_File_Path =  os.path.join(settings_Dir, 'settings.ini')


#================= GET SETTINGS FROM EMAIL SECTION IN settings.ini FILE ==============
def read_Email_Settings():

    try:
        config = ConfigParser.ConfigParser()
        config.optionxform=str   #By default config returns keys from Settings file in lower case. This line preserves the case for keys
        config.read(settings_File_Path)

        global FROM_ADD
        global USERNAME
        global PASSWORD
        global SMTP_SERVER
        global SMTP_PORT
        
        SMTP_SERVER = config.get("EMAIL","SMTP_ADD")
        SMTP_PORT = config.get("EMAIL","SMTP_PORT")
        FROM_ADD = config.get("EMAIL","FROM_ADD")
        USERNAME = config.get("EMAIL","USERNAME")
        PASSWORD = config.get("EMAIL","PASSWORD")

    except Exception as error_msg:
        print ("Error while trying to read SMTP/EMAIL Settings.")
        print ("Error: " + str(error_msg))
#=====================================================================================

read_Email_Settings()


class Class_eMail():
    
    def __init__(self):
        self.session = smtplib.SMTP_SSL(SMTP_SERVER, SMTP_PORT)
        self.session.ehlo()
        self.session.login(USERNAME, PASSWORD)

        
    def initialise_Mail_Body(self, To_Add, Subject, multipart_type):
        #Prepare Mail Body
        Mail_Body = MIMEMultipart(multipart_type)
        Mail_Body['From'] = FROM_ADD
        Mail_Body['To'] = To_Add
        Mail_Body['Subject'] = Subject
        return Mail_Body
    
    
    #Call this to send plain text emails.
    def send_Text_Mail(self, To_Add, Subject, txtMessage):
        Mail_Body = self.initialise_Mail_Body(To_Add, Subject, 'related')
        #Attach Mail Message
        Mail_Msg = MIMEText(txtMessage, 'plain')
        Mail_Body.attach(Mail_Msg)
        #Send Mail
        self.session.sendmail(FROM_ADD, [To_Add], Mail_Body.as_string())
    
    
    #Call this to send HTML emails.
    def send_HTML_Mail(self, To_Add, Subject, htmlMessage):
        Mail_Body = self.initialise_Mail_Body(To_Add, Subject, 'related')
        #Attach Mail Message
        Mail_Msg = MIMEText(htmlMessage, 'html')
        Mail_Body.attach(Mail_Msg)
        #Send Mail
        self.session.sendmail(FROM_ADD, [To_Add], Mail_Body.as_string())

    def send_Image_Mail(self, To_Add, Subject, image):
        msgRoot = self.initialise_Mail_Body(To_Add, Subject, 'related')
        # Encapsulate the plain and HTML versions of the message body in an
        # 'alternative' part, so message agents can decide which they want to display.
        msgAlternative = MIMEMultipart('alternative')
        msgRoot.attach(msgAlternative)

        msgText = MIMEText('Someone is in the house')
        msgRoot.attach(msgText)

        # We reference the image in the IMG SRC attribute by the ID we give it below
        msgText = MIMEText('<h1><center>Someone is in the house! Last detection</h1></center><br><img src="cid:image1" width="1024" height="580"/>', 'html')
        msgAlternative.attach(msgText)

        # This example assumes the image is in the current directory
        #fp = open('Detection_latest.jpg', 'rb')
        #msgImage = MIMEImage(fp.read())
        #fp.close()
        msgImage = MIMEImage(image)

        # Define the image's ID as referenced above
        msgImage.add_header('Content-ID', '<image1>')
        msgRoot.attach(msgImage)

        self.session.sendmail(FROM_ADD, [To_Add], msgRoot.as_string())
        

    def __del__(self):
        self.session.close()
        del self.session





