import serial
import time
import sys
import datetime
import binascii

class pmsA003():
    def __init__(self, dev):
        self.serial = serial.Serial(dev, baudrate=9600, timeout=1)
        self.arraysize = 32
        self.data = bytearray(self.arraysize)
        print(self.data)
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
            s1 = self.serial.read(1)
            s2 = self.serial.read(1)
            if s1 == b'\x42' and s2 == b'\x4d':
                payload = self.serial.read(30)
                #print(s1,s2)
                self.data = bytearray(s1 + s2 + payload)
                if self.verify_data():
                    if data_type == 'dict':
                        return self._PMdata_dict()
                    if data_type == 'list':
                        return self._PMdata_list()
                else:
                    if data_type == 'dict':
                        return {'time': 0, 'pm1.0': 0, 'pm2.5' : 0, 'pm10.0' : 0}
                    if data_type == 'list':
                        return [0, 0, 0, 0]

    def _PMdata_dict(self):
        d = {}
        d['time'] = datetime.datetime.now()
        d['pm1.0'] = self.data[10] * 256 + self.data[11]
        d['pm2.5'] = self.data[12] * 256 + self.data[13]
        d['pm10.0'] = self.data[14] * 256 + self.data[15]
        return d
    
    def _PMdata_list(self):
        d = [0,0,0,0]
        d[0] = datetime.datetime.now()
        d[1] = self.data[10] * 256 + self.data[11]
        d[2] = self.data[12] * 256 + self.data[13]
        d[3] = self.data[14] * 256 + self.data[15]
        return d


#print ("PMS7003 measurements starting...")
#con = pmsA003('/dev/ttyS0')

#while True:
#    if __name__ == '__main__':
        
#        PM = con.read_data()
#        print(PM)

