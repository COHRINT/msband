#/usr/bin/python2
'''
Proof of concept for talking RFCOMM to the Microsoft Band
Based on https://github.com/brandonasuncion/Reverse-Engineering-Bluetooth-Protocols/blob/master/radar.py
'''
import time
from bluetooth import *
import sys
import struct
import math


#addr = "58:82:a8:cb:51:87"
#addr = "4c:0b:be:fd:e3:f9"
class BandConnection(object):
    cmdUuid = "a502ca97-2ba5-413c-a4e0-13804e47b38f"
    dataUuid = "c742e1a2-6320-5abc-9643-d206c677e580"
    
    def __init__(self, addr, *args):
        #Search for available bands
        self.hasConnection = False
        self.cmdSock = self.openChannel(self.cmdUuid, addr)
        if self.cmdSock is None:
            print 'Unable to open the band at: ', addr
            self.dataSock = None
            return

        self.dataSock = self.openChannel(self.dataUuid, addr)
        self.hasConnection = True
        
        return

    
    def findBands(self):
        #Once paired, the band doesn't show up as a discoverable
        
        #devices = discover_devices(lookup_names = True)
        #print 'Found devices:', devices

        #service_matches = find_service( uuid = self.cmdUuid, address = addr)
        #print 'Found bands:', service_matches
        return
    
    def openChannel(self, uuid,addr):
        service_matches = find_service( uuid = uuid, address = addr )

        if len(service_matches) == 0:
            print("MSBand: couldn't find the service")
            return None

        first_match = service_matches[0]
        port = first_match["port"]
        name = first_match["name"]
        host = first_match["host"]

        print("BandConnection:Channel: connecting to \"{}\" on {} port {}".format(name, host, port))

        try:
            sock=BluetoothSocket( RFCOMM )
            sock.connect((host, port))
            print 'BandConnection: Good connection!'
        except btcommon.BluetoothError:
            print("BandConnection: Unable to connect")
            return None

        return sock
    
    def close(self):
        if not self.cmdSock is None:
            self.cmdSock.close()
        if not self.dataSock is None:
            self.dataSock.close()
            
class BandCommand(object):
    def __init__(self, command, dataStage, extendedArg = None):
        self.magic = 0x2ef9
        self.length = 8
        self.command = command
        self.dataStage = dataStage
        self.extended = extendedArg
        
    def packCommand(self):
        #LE, packed alignment
        if self.extended is not None:
            return struct.pack('<BHHI%ds' % len(self.extended), self.length+len(self.extended), self.magic,
                           self.command, self.dataStage, self.extended)
        else:
            return struct.pack('<BHHI', self.length, self.magic,
                           self.command, self.dataStage)

class BandPushPacketType(object):
    RemoteSubscription = 0x01
    
class BandSensors(object):
    Heartrate = 0x10
    GSR = 0x0f
    SkinTemp = 0x14
    AmbientLight = 0x19
    AccelGyro16ms = 0x31
    DeviceContact = 0x23
    
    @classmethod
    def dumpHex(self, buf):
         print 'Hex: ',  " ".join([hex(ord(i)) for i in buf])

    @classmethod
    def parseRemoteSubscription(cls, buf, actions):
        (subType, missedData, data) = struct.unpack('<BB%ds' % (len(buf) - 2), buf)
        if subType == BandSensors.Heartrate:
            cls.parseHeartrate(data, actions)
        elif subType == BandSensors.GSR:
            cls.parseGSR(data, actions)
        elif subType == BandSensors.SkinTemp:
            cls.parseSkinTemp(data, actions)
        elif subType == BandSensors.AmbientLight:
            cls.parseAmbientLight(data, actions)
        elif subType == BandSensors.AccelGyro16ms:
            cls.parseAccelGyro(data, actions)
        elif subType == BandSensors.DeviceContact:
            cls.parseDeviceContact(data, actions)
        else:
            print 'Unknown sensor type: ', hex(subType)

    @classmethod 
    def parseHeartrate(cls, buf, actions):
        #print 'Parsing heartrate data'
        #Parse the short off the front that has the sampleSize
        (sampleLength, sampData) = struct.unpack('<H%ds' % (len(buf) - 2), buf)
       
        #cls.dumpHex(sampData)
        (rate, qual, extra) = struct.unpack('<BB%ds' % (len(sampData) - 2), sampData)

        hr = {'rate': rate, 'quality': qual, 'locked': qual >= 6}
        actions.onHeartrate(hr)
        
    @classmethod 
    def parseGSR(cls, buf, actions):
        #print 'Parsing GSR data'
        #Parse the short off the front that has the sampleSize
        (sampleLength, sampData) = struct.unpack('<H%ds' % (len(buf) - 2), buf)
        if len(sampData) < 7:
            print 'Malformed packet:', cls.dumpHex(sampData)
        else:
            (status, resistance, capcount, data) = struct.unpack('<BLH%ds' % (len(sampData)-7), sampData)
            gsr = {'status': status, 'resistance': resistance, 'capcount': capcount}
            actions.onGSR(gsr)
        
    @classmethod 
    def parseSkinTemp(cls, buf, actions):
        #print 'Parsing skin temp data'
        convFactor = 0.01
        #output is deg C
        
        #Parse the short off the front that has the sampleSize
        
        #Weird marshalas booleans... @ 4 bytes apiece for true/false data
        (sampleLength, sampData) = struct.unpack('<h%ds' % (len(buf) - 2), buf)
        #cls.dumpHex(sampData)

        (skin, data) = struct.unpack('<H%ds' % (len(sampData) - 2), sampData)
        
        temp = {'temperature': skin*convFactor}
        
        actions.onSkinTemp(temp)
        
    @classmethod 
    def parseAmbientLight(cls, buf, actions):
        #print 'Parsing ambient light data'
        #Parse the short off the front that has the sampleSize
        (sampleLength, sampData) = struct.unpack('<H%ds' % (len(buf) - 2), buf)
        #cls.dumpHex(sampData)
        (lux, leftovers) = struct.unpack('<B%ds' % (len(sampData) - 1), sampData)
       
        ambient = {'lux': lux}
        actions.onAmbientLight(ambient)

        
    @classmethod
    def norm(cls, ax,ay,az):
        accel_m_s2 = 1.0
        return math.sqrt((ax*accel_m_s2)**2 + (ay*accel_m_s2)**2 + (az*accel_m_s2)**2)
    
    @classmethod 
    def parseAccelGyro(cls, buf, actions):
        
        #Fixed point: Multiply accel by 0.000244140625 to get m/s^2
        #Multiply gyro by 0.030487804878048783 to get deg/s

        accel_m_s2 = 0.000244140625*9.80665 #the default constant is in g, not m/s^2
        gyro_deg_s = 0.030487804878048783 / 180.0 * math.pi #convert to rad/s
        
        #print 'Parsing accel/gyro data'
        #Parse the short off the front that has the sampleSize
        (sampleLength, sampData) = struct.unpack('<H%ds' % (len(buf) - 2), buf)
        numMeasures = len(sampData) / 12

        #print 'Found %d measurements' % numMeasures
        samples = list()
        for i in range(0, numMeasures):
            #cls.dumpHex(sampData)
            (ax, ay, az, gx, gy, gz, sampData) = struct.unpack('<hhhhhh%ds' % (len(sampData) - 12), sampData)  
            imu = {'ax':ax*accel_m_s2, 'ay':ay*accel_m_s2, 'az':az*accel_m_s2,
                       'gx':gx*gyro_deg_s, 'gy':gy*gyro_deg_s, 'gz':gz*gyro_deg_s}

            
         
            #print 'Est constant:', 9.80665 / cls.norm(ax, ay, az)
         
            samples.append(imu)            
        actions.onAccelGyro(samples)
        
    @classmethod 
    def parseDeviceContact(cls, buf, actions):
        print 'Parsing device contact'
        #Parse the short off the front that has the sampleSize
        (sampleLength, sampData) = struct.unpack('<H%ds' % (len(buf) - 2), buf)
        (wornState, fromGSR, fromHR, data) = struct.unpack('<BBB%ds' % (len(sampData) - 3), sampData)
                                                           
        #cls.dumpHex(buf)
        contact = {'state': wornState, 'fromGSR' : fromGSR, 'fromHR' : fromHR}
        
        actions.onDeviceContact(contact)
    
           
        
class BandDataActions(object):
    #Subclass this to implement custom actions
    #Each sensor gets a dict of the relevant dataclass to do something meaningful with
    
    def __init__(self):
        pass
    def onHeartrate(self, hr):
        print 'Got heartrate data:', hr
        
    def onGSR(self, gsr):
        print 'Got GSR:', gsr
        
    def onSkinTemp(self, skinTemp):
        print 'Got temp:', skinTemp
        
    def onAmbientLight(self, ambient):
        print 'Got ambient:', ambient
        
    def onAccelGyro(self, imu):
        print 'Got imu data:', imu

    def onDeviceContact(self, contact):
        print 'Got contact:', contact
    
class BandSubscribeParams(object):
    def __init__(self, sensorType, isPassive=0):
        self.sensorType = sensorType
        self.isPassive = isPassive
    def pack(self):
        return struct.pack('<BI', self.sensorType, self.isPassive)
    
class BandStatus(object):
    #Given the 6-byte status response, parse into something meaningful
    
    def __init__(self, status):
        (self.packetType, self.status) = struct.unpack('<HI', status)
        self.parseStatus()

    def parseStatus(self):
        #PacketType is 0xa6fe: this looks like a magic number for the status packet type
        self.code = self.status & 0x00ffff
        self.facility = (self.status >> 16) & 0x07ff
        self.reserved = (self.status >> 27) & 0x0f
        self.customer = (self.status >> 30) & 0x01
        self.severity = (self.status >> 31) & 0x01

class BandCargo(object):
    def __init__(self, conn=None):
        self.hasConnection = False
        
        if not (conn is None):
            self.cmdSock = conn.cmdSock
            self.dataSock = conn.dataSock
            self.hasConnection = True
            
    def dumpHex(self, buf):
        print 'Hex: ',  " ".join([hex(ord(i)) for i in buf])

    def decodeCommand(self, cmd):
        #The command is packed - split up and return as a tuple
        category = (cmd & 0xff00) >> 8
        isTX = (cmd & 0x0080) >> 7
        index = (cmd & 0x007f)
        return (category, isTX, index)
    
    def encodeCommand(self, cat, isTx, index):
        cmd = cat << 8
        cmd |= isTx << 7
        cmd |= index
        return cmd
    
    def cmdSendRecvAck(self, command):
        self.cmdSock.send(command.packCommand())
        recv = self.cmdSock.recv(64)
        statBuf = self.cmdSock.recv(10)
        stat = BandStatus(statBuf)
        if stat.status <> 0:
            print 'Band Command Debug:'
            print 'Command:', command
            print 'Recv Raw:', " ".join([hex(ord(i)) for i in recv])
            print 'Status: Code:', stat.code, ' Facility:', stat.facility
            
        return recv

    def cmdSendRecv(self, command):
        self.cmdSock.send(command.packCommand())
        recv = self.cmdSock.recv(64)
        #print 'Got ', len(recv), ' bytes:', " ".join([hex(ord(i)) for i in recv])
        return recv

    def subscribe(self, sensor):
        #Set up the commands to get some sensor data
        params = BandSubscribeParams(sensor)
        subCmd = BandCommand(self.encodeCommand(143, 0, 0), 0, params.pack())
        #print 'Sub cmd:', " ".join([hex(ord(i)) for i in subCmd.packCommand()])
        self.cmdSendRecv(subCmd)
        
    def setupBand(self):
        getSerial = BandCommand(self.encodeCommand(120, 1, 8), 0xc)
        
        self.serNum = self.cmdSendRecvAck(getSerial)
        print 'Serial number:', self.serNum
        self.subscribe(BandSensors.AmbientLight)
        self.subscribe(BandSensors.Heartrate)
        self.subscribe(BandSensors.GSR)
        self.subscribe(BandSensors.SkinTemp)
        self.subscribe(BandSensors.AccelGyro16ms)
        self.subscribe(BandSensors.DeviceContact)
        print 'Sensors subscribed'
        
    def readData(self, actions):
        #When the device goes out of range, this returns bluetooth.btcommon.BluetoothError 110
        #need to implement reconnects - and resubscription, while keeping the ROS publishers from destruction
        
        recv = self.dataSock.recv(1024)
        #print 'Data: ', len(recv), ' bytes:', " ".join([hex(ord(i)) for i in recv])

        #Data appear to come in nice packets already - no need to assemble a buffer
        if len(recv) < 6:
            print 'Short read:'
            self.dumpHex(recv)
            return
        
        (packetType, packetLength, data) = struct.unpack('<HL%ds' % (len(recv) - 6), recv)
        #self.dumpHex(data)
        if packetType == BandPushPacketType.RemoteSubscription:
            BandSensors.parseRemoteSubscription(data, actions)
        else:
            print 'Unknown data packet type:', hex(packetType)
            self.dumpHex(recv)
            
    def vibrate(self):
        vib = BandCommand(self.encodeCommand(154, 0, 0), 0x0, chr(20))
        #self.dumpHex(vib.packCommand())
        self.cmdSendRecv(vib)
        
    def startCortana(self):
        cort = BandCommand(self.encodeCommand(221, 0, 0), 0x0)
        self.cmdSendRecv(cort)
        cort = BandCommand(self.encodeCommand(221, 0, 1), 0x0c)
        self.cmdSendRecv(cort)

    def getOOBEComplete(self):
        oobe = BandCommand(self.encodeCommand(202, 1, 19), 0x0)
        self.cmdSendRecv(oobe)
        
    def setOOBEComplete(self):
        oobe = BandCommand(self.encodeCommand(202, 0, 1), 0x1)
        self.cmdSendRecv(oobe)
        
    def setOOBEClear(self):
        oobe = BandCommand(self.encodeCommand(202, 0, 0), 0x0)
        self.cmdSendRecv(oobe)

def main():
    addr = "58:82:a8:cb:51:87"
    #addr = "4c:0b:be:fd:e3:f9"

    b = BandConnection(addr = addr)
    if not b.hasConnection:
        print 'Unable to establish connection'
        sys.exit(0)
        
    #b = None
    c = BandCargo(b)

    actions = BandDataActions()
    c.setupBand()

    while True:
        c.readData(actions)
    
    b.close()
    
    
if __name__ == "__main__":
    main()
