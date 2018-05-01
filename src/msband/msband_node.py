#!/usr/bin/env python2
import sys, serial
import rospy
import time
from sensor_msgs.msg import Imu

from msband.msg import Heartrate, GSR, AmbientLight, DeviceContact, SkinTemp
import std_msgs.msg

from msband_dev import *

import pdb



class ROSActions(BandDataActions):
    #Subclass this to implement custom actions
    #Each sensor gets a dict of the relevant dataclass to publish to its publishers
    
    def __init__(self):
        super(ROSActions, self).__init__()
        self.hr_pub = rospy.Publisher('heartrate', Heartrate, queue_size=10)
        self.gsr_pub = rospy.Publisher('gsr', GSR, queue_size=10)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
        self.contact_pub = rospy.Publisher('contact', DeviceContact, queue_size=10)
        self.temp_pub = rospy.Publisher('temp', SkinTemp, queue_size=10)
        self.light_pub = rospy.Publisher('light', AmbientLight, queue_size=10)
        
    def onHeartrate(self, hr):
        #print 'Got heartrate data:', hr

        msg = Heartrate()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.rate = hr['rate']
        msg.quality = hr['quality']
        msg.locked = hr['locked']
        self.hr_pub.publish(msg)
        
    def onGSR(self, gsr):
        #print 'Got GSR:', gsr
        msg = GSR()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.status = gsr['status']
        msg.resistance = gsr['resistance']
        msg.capcount = gsr['capcount']
        self.gsr_pub.publish(msg)

        
    def onSkinTemp(self, skinTemp):
        #print 'Got temp:', skinTemp
        msg = SkinTemp()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.temp = skinTemp['temperature']
        self.temp_pub.publish(msg)
        
    def onAmbientLight(self, ambient):
        #print 'Got ambient:', ambient
        msg = AmbientLight()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.lux = ambient['lux']
        self.light_pub.publish(msg)
        
    def onAccelGyro(self, imu):
        #print 'Got imu data:', imu

        #Imu might provide multiple samples per dict
        for i in range(0, len(imu)):
            msg = Imu()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = rospy.Time.now()
            msg.angular_velocity.x = imu[i]['gx']
            msg.angular_velocity.y = imu[i]['gy']
            msg.angular_velocity.z = imu[i]['gz']
            msg.linear_acceleration.x = imu[i]['ax']
            msg.linear_acceleration.y = imu[i]['ay']
            msg.linear_acceleration.z = imu[i]['az']

            self.imu_pub.publish(msg)
        
    def onDeviceContact(self, contact):
        #print 'Got contact:', contact
        msg = DeviceContact()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.state = contact['state']
        msg.fromGSR = contact['fromGSR']
        msg.fromHR = contact['fromHR']
        
        self.contact_pub.publish(msg)

if __name__ == "__main__":
    #parser = argparse.ArgumentParser(description="Microsoft Band ROS node")
    #parser.add_argument("addr", help="Bluetooth device address of the band.")

    #args = parser.parse_args()
    if len(sys.argv) < 2:
        print 'Usage:', sys.argv[0], ' <band bluetooth addr>'
        sys.exit(0)
        
    print 'Attempting to find band at: ', sys.argv[1]
    bandAddr = sys.argv[1]
    
    try:

        conn = BandConnection(bandAddr)
        if not conn.hasConnection:
            print 'Unable to open channels to band'
            sys.exit(0)
            
        cargo = BandCargo(conn)
        cargo.setupBand()
        rospy.init_node('msband_node', anonymous=True)

        actions = ROSActions()
        while not rospy.is_shutdown():
            cargo.readData(actions)

        conn.close()
        
    except rospy.ROSInterruptException:
        pass


