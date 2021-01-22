#!/usr/bin/env python

import rospy
import time
import math
from frl_vehicle_msgs.msg import UwGliderCommand

import matplotlib.pyplot as plt
#import pandas as pd

def command(startTime):
    while not rospy.is_shutdown():
        if rospy.get_time()-startTime >= 1:  # initiate after 2 seconds

            # dummy initial command
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            pub.publish(command)
            time.sleep(1)

            print("\n----- Ascend with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle)------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = -0.01
            command.target_pumped_volume = 0.5
            command.rudder_control_mode = 1
            command.target_heading = math.pi/2
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            print("\n----- Dive with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle)------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = 0.01
            command.target_pumped_volume = -0.5
            command.rudder_control_mode = 1
            command.target_heading = math.pi/2
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            print("\n----- Dive with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle) + Thruster------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = 0.005
            command.target_pumped_volume = -0.5
            command.rudder_control_mode = 1
            command.target_heading = math.pi/3
            command.motor_cmd_type = 1
            command.target_motor_cmd = 0.005
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            print("\n----- Dive with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle) + Thruster------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = -0.01
            command.target_pumped_volume = -0.5
            command.rudder_control_mode = 1
            command.target_heading = math.pi/4
            command.motor_cmd_type = 1
            command.target_motor_cmd = 0.01
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            # Shutdown
            rospy.signal_shutdown('\n\nDONE!')

        # rate.sleep()

if __name__ == '__main__':
    try:
        # start node
        pub = rospy.Publisher('/glider_hybrid_whoi/direct_kinematics/UwGliderCommand', UwGliderCommand, queue_size=10)
        rospy.init_node('commander', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        startTime = rospy.get_time()

        # send command
        command(startTime)

    except rospy.ROSInterruptException:
        pass
