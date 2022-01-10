#!/usr/bin/env python
'''
Publishes current lat/lon as NMEA msg for Fledermaus Vessel Manager

'''

# System imports
from math import *

# ROS/Gazebo imports
import rospy
import tf
from frl_vehicle_msgs.msg import UwGliderStatus

# For UDP connection
import socket

# For NMEA msg format
import pynmea2
# ---- NMEA standard string ----- # (does not have roll, pitch, heading)
# $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
# 1    = UTC of Position
# 2    = Latitude
# 3    = N or S
# 4    = Longitude
# 5    = E or W
# 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
# 7    = Number of satellites in use [not those in view]
# 8    = Horizontal dilution of position
# 9    = Antenna altitude above/below mean sea level (geoid)
# 10   = Meters  (Antenna height unit)
# 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
#        mean sea level.  -=geoid is below WGS-84 ellipsoid)
# 12   = Meters  (Units of geoidal separation)
# 13   = Age in seconds since last update from diff. reference station
# 14   = Diff. reference station ID#
# 15   = Checksum

# ---- NMEA custom string ----- # (includes roll, pitch, heading)
# $C&C,SIM,00:00:00.0,541173.578779,167895.660529,296.088928,0.0,0.0,-2.564224,0.0
# Vehicle Name -> A string. In the example the name is SIM.
# Time -> hh:mm:ss.s for hours, minutes, seconds and fractional secs.
# X-Coordinate -> Can be a projected easting or decimal geographic longitude
# Y-Coordinate -> Can be a projected northing or decimal geographic latitude
# heading -> in degrees [0, 360]
# pitch -> in degrees ( +ve is pitch up )
# roll -> in degrees ( +ve is roll to starboard )
# height -> in units of the data (+ve is up)
# altitude -> in units of the data (currently not used)
# msg = '$C&C,' + 'PV,' + str(sTohhmmss2(sim_time)) + ',' +  str(decdeg2dms(-longitude)) \
#       + ',' +  str(decdeg2dms(latitude)) + ',' + str(90) + ',' + '0.2, 0.0, -1, 0.0'

def ddToddm(deg):
     d = int(deg)
     dm = abs(deg - d) * 60
     return "%i%f" % (d, dm)

def sTohhmmss(seconds):
    hours = seconds // (60*60)
    seconds %= (60*60)
    minutes = seconds // 60
    seconds %= 60
    return "%02i%02i%02i" % (hours, minutes, seconds)

def sTohhmmss2(seconds):
    hours = seconds // (60*60)
    seconds %= (60*60)
    minutes = seconds // 60
    seconds %= 60
    return "%02i:%02i:%02i" % (hours, minutes, seconds)

class Node():
    def __init__(self, host_ip, port):
        # Keep track of time and depth
        self.t0 = rospy.get_time()
        while self.t0 < 0.01:
            rospy.logwarn("Waiting for ROS tme to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()
            self.t0_init = self.t0

        # Init messages
        self.sim_msg = UwGliderStatus()

        # Initiate socket for UDP connection
        self.UDP_IP = host_ip
        self.UDP_PORT = port  # default port of the Vessel Manager
        self.sock = socket.socket(socket.AF_INET, # Internet
                                  socket.SOCK_DGRAM) # UDP

        # Subscribe UwGliderStatus
        self.sub_imu = rospy.Subscriber("status", UwGliderStatus, self.callback_UwGliderStatus)

        # Print msg
        rospy.loginfo("NMEA msg publisher for Fledermaus is to = " + \
                repr(self.UDP_IP) + " with port " + repr(self.UDP_PORT))

    def callback_UwGliderStatus(self, data):
        self.sim_msg = data

    def __del__(self):
        #print("Closing server socket:", self.sock)
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()

    def publish_NMEA(self):
        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-3:
            # rospy.logwarn("Timestep is too small (%f) - skipping this update"
            #               %dt)
            return
        sim_time = now

        # Read UwGliderStatus msg
        latitude = self.sim_msg.latitude
        longitude = self.sim_msg.longitude
        depth = self.sim_msg.depth
        roll = self.sim_msg.roll
        pitch = self.sim_msg.pitch
        heading = self.sim_msg.heading

        # msg = pynmea2.GGA('PV', 'GGA', \
        #     (str(sTohhmmss(sim_time)), \
        #     str(ddToddm(latitude)), 'N', \
        #     str("0" + ddToddm(-longitude)), 'W', \
        #     '1', '04', '', \
        #     str(depth), 'M', \
        #     '', 'M', '', '0000'))

        msg = '$C&C,' + 'HYBRID_GLIDER,' + str(sTohhmmss2(sim_time)) + ',' \
            +  str(longitude) + ',' +  str(latitude) + ',' + str(heading) + ',' \
            + str(-pitch) + ',' + str(roll) + ',' + str(depth) + '0.0'

        MESSAGE = str(msg)
        self.sock.sendto(MESSAGE.encode(), (self.UDP_IP, self.UDP_PORT))

        return

if __name__ == '__main__':

    # Start node
    rospy.init_node('nmea_publisher')

    # Update rate
    update_rate = rospy.get_param('~update_rate', 5.0)

    # Get host ip address and port
    host_ip = rospy.get_param('~host_ip', "128.128.62.4")
    port = rospy.get_param('~port', 30362)

    # Initiate node object
    node=Node(host_ip, port)

    # Spin
    r = rospy.Rate(update_rate)
    try:
        while not rospy.is_shutdown():
            node.publish_NMEA()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
