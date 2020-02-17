import serial
import time
import sys
import datetime
import binascii

class pmsA003():
    def __init__(self, dev):
        self.serial = serial.Serial(dev, baudrate=9600, timeout=3)
        self.arraysize = 32
        self.data = bytearray(self.arraysize)
        self.pm_valid = [False, False, False]
        
    def __exit__(self, exc_type, exc_value, traceback):
        print('pmsA003 serial closing')
        self.serial.close()


    def setIdle(self):
        idlecmd = b'\x42\x4d\xe4\x00\x00\x01\x73'
        ary = bytearray(idlecmd)
        self.serial.write(ary)

    def setNormal(self):
        normalcmd = b'\x42\x4d\xe4\x00\x01\x01\x74'
        ary = bytearray(normalcmd)
        self.serial.write(ary)

    def verify_data(self):

        crc = self.data[30] * 256 + self.data[31]
        crc_calc = 0
        for i in range(len(self.data)-2):
            crc_calc += self.data[i]
        
        if crc == crc_calc:
            
            self.pm_valid = [True, True, True]
            pm = [0,0,0]
            
            pm[0] = self.data[10] * 256 + self.data[11]
            pm[1] = self.data[12] * 256 + self.data[13]
            pm[2] = self.data[14] * 256 + self.data[15]

            if pm[0] < 0 or pm[0] > 500:
                self.pm_valid[0] = False
            if pm[1] < 0 or pm[1] > 500:
                self.pm_valid[1] = False
            if pm[2] < 0 or pm[2] > 500:
                self.pm_valid[2] = False
            
            return True
        
        else:
            return False

    def getFalseValues(self, data_type):
        if data_type == 'dict':
            return {'time': datetime.datetime.now(), 'pm1.0': None, 'pm2.5' : None, 'pm10.0' : None}
        
        if data_type == 'list':
            return [datetime.datetime.now(), None, None, None]


    def read_data(self, retry_count = 32, data_type = 'list'):

        retries_left = retry_count

        while retries_left > 0:

            s1 = self.serial.read(1)
            if s1 == b'\x42':
                s2 = self.serial.read(1)
                if s2 == b'\x4d':
                    payload = self.serial.read(30)
                    self.data = bytearray(s1 + s2 + payload)
                    if self.verify_data():
                        if data_type == 'dict':
                            return self._PMdata_dict()
                        if data_type == 'list':
                            return self._PMdata_list()
                    else:
                        return self.getFalseValues(data_type)
            else: 
                retries_left -= 1

        print('pmsA003 read error')
        return self.getFalseValues(data_type)  

    def _PMdata_dict(self):

        d = {}

        d['time'] = datetime.datetime.now()

        if self.pm_valid[0]:
            d['pm1.0'] = self.data[10] * 256 + self.data[11]

        if self.pm_valid[1]:
            d['pm2.5'] = self.data[12] * 256 + self.data[13]

        if self.pm_valid[2]:
            d['pm10.0'] = self.data[14] * 256 + self.data[15]

        return d
    
    def _PMdata_list(self):

        d = [0,0,0,0]

        d[0] = datetime.datetime.now()

        if self.pm_valid[0]:
            d[1] = self.data[10] * 256 + self.data[11]
        
        if self.pm_valid[1]:
            d[2] = self.data[12] * 256 + self.data[13]
        
        if self.pm_valid[2]:
            d[3] = self.data[14] * 256 + self.data[15]

        return d


#print ("PMS7003 measurements starting...")
#con = pmsA003('/dev/ttyS0')

#while True:
#    if __name__ == '__main__':
#        PM = con.read_data()
#        print(PM)

