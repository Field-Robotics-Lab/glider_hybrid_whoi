#!/usr/bin/env python
'''
Node to implement dead reckoning estimation.

'''

# System imports
from math import *

# ROS/Gazebo imports
import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import FluidPressure
from frl_vehicle_msgs.msg import UwGliderStatus

# import GDAL
# from osgeo import osr

def decibars2depth_salt(decibars,latitude):
    '''
    From Seabird application note 69 (AN69).
    Return depth in meters
    Latitude is given in units of decimal degrees
    '''
    p = decibars
    x = ( sin(latitude/57.29578) )**2
    g = 9.780318*(1.0+( 5.2788e-3 + 2.36e-5 * x) * x ) + 1.092e-6 * p
    d = ((((-1.82e-15 * p+2.279e-10)*p-2.2512e-5)*p+9.72659)*p) / g
    return d

def pascals2depth(pascals):
    '''
    Wrapper function with latitude hardcoded for now
    '''
    decibars = pascals/1.0e5
    return decibars2depth_salt(decibars, 41.0)

def kilopascals2depth(kilopascal):
    '''
    kilopascal per meter hardcoded
    '''
    standardPressure = 101.325
    kPaPerM=9.80638
    return (kilopascal-101.325)/kPaPerM

def  mdeglat(lat):
    '''
    Provides meters-per-degree latitude at a given latitude
    From AlvinXY

    Args:
      lat (float): latitude
    Returns:
      float: meters-per-degree value
    '''
    latrad = lat*2.0*pi/360.0

    dy = 111132.09 - 566.05 * cos(2.0*latrad) \
         + 1.20 * cos(4.0*latrad) \
         - 0.002 * cos(6.0*latrad)
    return dy

def mdeglon(lat):
    '''
    Provides meters-per-degree longitude at a given latitude
    From AlvinXY

    Args:
      lat (float): latitude in decimal degrees
    Returns:
      float: meters per degree longitude
    '''
    latrad = lat*2.0*pi/360.0

    dx = 111415.13 * cos(latrad) \
         - 94.55 * cos(3.0*latrad) \
	       + 0.12 * cos(5.0*latrad)
    return dx

class Node():
    def __init__(self, angleofattack_deg, write_flag):

        # Current estimate
        self.dr_msg = NavSatFix()
        self.dr_msg.latitude = 0.0
        self.dr_msg.longitude = 0.0

        # # Transform using GDAL
        # tsrs = osr.SpatialReference()
        # tsrs.ImportFromEPSG(26987)
        # srs = osr.SpatialReference()
        # srs.ImportFromEPSG(4326)
        # latlonTrans = osr.CoordinateTransformation(srs,tsrs)
        # self.xCoord, self.yCoord, z = latlonTrans.TransformPoint(init_lon,init_lat)

        # Fixec AOA
        self.angleofattack_rad = angleofattack_deg * pi/180.0

        # Keep track of time and depth
        self.t0 = rospy.get_time()
        while self.t0 < 0.01:
            rospy.logwarn("Waiting for ROS tme to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()
            self.t0_init = self.t0
        self.d0 = None

        # Init messages
        self.imu_msg = Imu()
        self.pressure_msg = FluidPressure()
        self.gps_msg = None
        self.sim_msg = UwGliderStatus()

        # Pubs and subs
        self.sub_imu = rospy.Subscriber("imu", Imu, self.callback_imu)
        self.sub_pressure = rospy.Subscriber("pressure",
                                             FluidPressure,
                                             self.callback_pressure)
        self.sub_gps = rospy.Subscriber("fix", NavSatFix, self.callback_gps)
        self.pub_fix = rospy.Publisher("deadreckon", NavSatFix, queue_size=1)


    def callback_imu(self, data):
        self.imu_msg = data

    def callback_pressure(self, data):
        self.pressure_msg = data

    def callback_gps(self, data):
        self.gps_msg = data

    def update(self):
        '''
        Estimate
        '''
        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-3:
            # rospy.logwarn("Timestep is too small (%f) - skipping this update"
            #               %dt)
            return
        self.t0 = now

        # Convert fluid pressure to depth
        depth = kilopascals2depth(self.pressure_msg.fluid_pressure)

        # Need to intialize depth on first pass
        if self.d0 is None:
            self.d0 = depth
            # Need to intialize lat/lon on first pass
            self.sim_msg = rospy.wait_for_message("status", UwGliderStatus)
            self.dr_msg.latitude = self.sim_msg.latitude
            self.dr_msg.longitude = self.sim_msg.longitude
            print("DR estimation started. Initial lat/lon = " + \
                  repr(self.dr_msg.latitude) + "/" + repr(self.dr_msg.longitude))
            return

        # Convert IMU attitude to Euler
        q = (self.imu_msg.orientation.x,
             self.imu_msg.orientation.y,
             self.imu_msg.orientation.z,
             self.imu_msg.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(q)

        # If we have GPS (Surfaced) - just use that
        gpsFix = False
        if self.gps_msg is not None \
           and self.gps_msg.status.status == NavSatStatus.STATUS_SBAS_FIX:
            self.dr_msg.latitude = self.gps_msg.latitude
            self.dr_msg.longitude = self.gps_msg.longitude
            self.dr_msg.header.stamp = rospy.Time.now()
            self.dr_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            self.pub_fix.publish(self.dr_msg)
            self.gps_msg = None
            gpsFix = True

        if gpsFix == False:
          # Algo. from:
          # https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/3
          dz = depth - self.d0
          self.d0 = depth
          # Assume constant angle of attack
          # Note that coordinates are in ENU
          aoa = self.angleofattack_rad
          if rpy[1] < 0:
            glide_angle = rpy[1] - aoa
          else:
            glide_angle = rpy[1] + aoa

          if abs(rpy[1]) < pi/100.0:
              v_x = 0.0
              v_y = 0.0
          else:
              v_x = dz/(dt*tan(glide_angle))*sin(pi/2.0-rpy[2])
              v_y = dz/(dt*tan(glide_angle))*cos(pi/2.0-rpy[2])
          dx = v_x * dt
          dy = v_y * dt

          # Increment lat/lon
          self.dr_msg.latitude += dy/mdeglat(self.dr_msg.latitude)
          self.dr_msg.longitude += dx/mdeglon(self.dr_msg.latitude)

          # Transform using GDAL
          # self.xCoord += dx
          # self.yCoord += dy
          # srs = osr.SpatialReference()
          # srs.ImportFromEPSG(26987)
          # tsrs = osr.SpatialReference()
          # tsrs.ImportFromEPSG(4326)
          # coordTrans = osr.CoordinateTransformation(srs,tsrs)
          # self.dr_msg.longitude, self.dr_msg.latitude, dummy = coordTrans.TransformPoint(self.xCoord, self.yCoord)

          # Publish
          self.dr_msg.header.stamp = rospy.Time.now()
          self.pub_fix.publish(self.dr_msg)

        # Write Log
        if write_flag:
          write_file = open("/tmp/DRLog.csv", "a")
          write_file.write(repr(now) + "," + repr(depth) + "," + repr(self.dr_msg.latitude) + "," + repr(self.dr_msg.longitude) + "," + repr(gpsFix) + "\n")

        return

if __name__ == '__main__':

    # Start node
    rospy.init_node('deadreckoning_estimator')

    # Specified in degrees, positive is bow down
    angleofattack_deg = rospy.get_param('~angleofattack',  -4.0 )

    # Update rate
    update_rate = rospy.get_param('~update_rate', 5.0)

    # Open a file to write log
    write_flag = rospy.get_param('~writeLog', 0)
    if write_flag:
      write_file = open("/tmp/DRLog.csv", "w")
      write_file.write("time,depth,lat,lon,gpsBool\n")

    # Initiate node object
    node=Node(angleofattack_deg, write_flag)

    # Spin
    r = rospy.Rate(update_rate)
    try:
        while not rospy.is_shutdown():
            node.update()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
