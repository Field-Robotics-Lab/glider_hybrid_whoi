#!/usr/bin/env python

import rospy
import time
from frl_vehicle_msgs.msg import UwGliderCommand

import matplotlib.pyplot as plt
import pandas as pd

def command(startTime):
    while not rospy.is_shutdown():
        if rospy.get_time()-startTime >= 1:  # initiate after 2 seconds
            
            # --- UwGliderCommand ---
            # 'header', 'pitch_cmd_type', 'target_pitch_value', 'motor_cmd_type', 'target_motor_cmd', 'rudder_control_mode', 'target_heading', 'rudder_angle', 'target_rudder_angle', 'target_pumped_volume'

            # dummy initial command
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            pub.publish(command)
            time.sleep(1)

            # 1st command
            print("\n----- 1st command ------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 2
            command.target_pitch_value = 0.9
            command.motor_cmd_type = 2
            command.target_motor_cmd = 6
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            # 2nd command
            print("\n----- 2nd command ------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 2
            command.target_pitch_value = 1.2
            command.motor_cmd_type = 2
            command.target_motor_cmd = 8
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            # 3rd command
            print("\n----- 3rd command ------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 2
            command.target_pitch_value = 0.2
            command.motor_cmd_type = 2
            command.target_motor_cmd = 4
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            # 4th command
            print("\n----- 4th command ------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 2
            command.target_pitch_value = 0.0
            command.motor_cmd_type = 2
            command.target_motor_cmd = 2
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            # 5th command
            print("\n----- 5th command ------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 2
            command.target_pitch_value = -0.2
            command.motor_cmd_type = 2
            command.target_motor_cmd = 4
            command.target_pumped_volume = 2.0
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)
            
            # 6th command
            print("\n----- 5th command ------")
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 2
            command.target_pitch_value = -0.4
            command.motor_cmd_type = 2
            command.target_motor_cmd = 1
            command.target_pumped_volume = 15.0
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            # Shutdown 
            rospy.signal_shutdown('\n\nDONE!')

        # rate.sleep()

def plot(startTime):
    
    time.sleep(5)

    # Read data
    header_list = ["t", "x", "y", "z", "p", "q", "r", "altitude"]
    log = pd.read_csv('/tmp/DirectKinematicsLog.csv', names = header_list, skiprows = 2)

    # limit data to time range where commands are sent
    # ?? 5 seconds delay. Needs investigation
    indexNames = log[log['t'] < startTime + 5].index
    log.drop(indexNames, inplace = True)
    indexNames = log[log['t'] > rospy.get_time() + 5].index
    log.drop(indexNames, inplace = True)

    # save dropped dataframe to csv
    log.to_csv('/tmp/DirectKinematicsLog.csv', index=False, header=True)

    # plot
    log.plot("t", ["x", "z", "q", "altitude"], subplots=True)
    plt.show()

if __name__ == '__main__':
    try:
        # start node
        pub = rospy.Publisher('/glider_hybrid_whoi/direct_kinematics/UwGliderCommand', UwGliderCommand, queue_size=10)
        rospy.init_node('commander', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        startTime = rospy.get_time()

        # send command
        command(startTime)

        # plot
        plot(startTime)
        print("\n\nRaw data is at /tmp/DirectKinematicsLog.csv")
        print("Copy and save now or it will be overwritten later")

    except rospy.ROSInterruptException:
        pass
