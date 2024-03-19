#!/usr/bin/env python
import os
import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse
#CMD = "ssh -i id_rsa_Palletizer er@er python3 btr_myPalletizer.py move_Work"
# CMD = "ssh palletizer-0 -l er python3 btr_myPalletizer.py move_Work"
CMD = "ssh palletizer-0 -l er -i /home/robotino/.ssh/id_rsa_pall python3 btr_myPalletizer.py move_Work"

def grab_Arm(data):
    cmd = CMD + " btr_g.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def release_Arm(data):
    cmd = CMD + " btr_r.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def pick_rs(data):
    cmd = CMD + " w_btr_pick_rs.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def put_rs(data):
    cmd = CMD + " w_btr_put_rs.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def looking_for_C0(data):
    cmd = CMD + " btr_looking_for_C0.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def grasp_C0(data):
    cmd = CMD + " btr_g_C0.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def scan_Arm(data):
    cmd = CMD + " btr_scan.txt"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def return_Empty(data):
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node("BTR_myCobot_ros")
    if len(sys.argv) > 1:
        if sys.argv[1] == True: # gazebo
            src00 = rospy.Service('/btr/move_g', Empty, return_Empty)
            src01 = rospy.Service('/btr/move_r', Empty, return_Empty)
            src02 = rospy.Service('/btr/pick_rs', Empty, return_Empty)
            src03 = rospy.Service('/btr/put_rs', Empty, return_Empty)
            src04 = rospy.Service('/btr/looking_for_C0', Empty, return_Empty)
            src05 = rospy.Service('/btr/move_g_C0',Empty, return_Empty)
            src06 = rospy.Service('/btr/move_s',Empty, return_Empty)
        else:
            src00 = rospy.Service('/btr/move_g', Empty, grab_Arm)
            src01 = rospy.Service('/btr/move_r', Empty, release_Arm)
            src02 = rospy.Service('/btr/pick_rs', Empty, pick_rs)
            src03 = rospy.Service('/btr/put_rs', Empty, put_rs)
            src04 = rospy.Service('/btr/looking_for_C0', Empty, looking_for_C0)
            src05 = rospy.Service('/btr/move_g_C0', Empty, grasp_C0)
            src06 = rospy.Service('/btr/move_s', Empty, scan_Arm)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

    print("Bye...")
