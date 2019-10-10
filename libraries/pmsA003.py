import serial
import time
import sys
import json
import datetime
import binascii

class pmsA003():
    def __init__(self, dev):
        self.serial = serial.Serial(dev, baudrate=9600, timeout=3)
    def __exit__(self, exc_type, exc_value, traceback):
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
        #TODO: better data validity checking, e.g. checksum
        #checksum = ord(_frame[-2]) << 8 | ord(_frame[-1])
        #calculated_checksum = HEAD_FIRST + HEAD_SECOND
        #for field in _frame[:-2]:
        #calculated_checksum += ord(field)
        #return checksum == calculated_checksum

        if not self.data:
            return False
        return True


    def read_data(self,data_type = 'list'):
        while True:
            b = self.serial.read(1)
            if b == b'\x42':
                data = self.serial.read(31)
                #if data[0] == b'\x4D':
                #    print('M')
                self.data = bytearray(b'\x42' + data)
                if self.verify_data():
                    if data_type == 'dict':
                        return self._PMdata_dict()
                    if data_type == 'list':
                        return self._PMdata_list()

    def _PMdata_dict(self):
        d = {}
        d['time'] = datetime.datetime.now()
        d['pm1.0'] = self.data[4] * 256 + self.data[5]
        d['pm2.5'] = self.data[6] * 256 + self.data[7]
        d['pm10.0'] = self.data[8] * 256 + self.data[9]
        return d
    
    def _PMdata_list(self):
        d = [0,0,0,0]
        d[0] = datetime.datetime.now()
        d[1] = self.data[4] * 256 + self.data[5]
        d[2] = self.data[6] * 256 + self.data[7]
        d[3] = self.data[8] * 256 + self.data[9]
        return d


#print ("PMS7003 measurements starting...")

#while True:
    #if __name__ == '__main__':
#    con = pmsA003('/dev/ttyS0')
#    PM = con.read_data()
#    print(PM[1], PM[2], PM[3])

