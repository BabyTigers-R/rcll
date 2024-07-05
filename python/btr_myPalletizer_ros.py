#!/usr/bin/env python
import os
import rospy
from std_srvs.srv import Empty, EmptyResponse
#CMD = "ssh -i id_rsa_Palletizer er@er python3 btr_myPalletizer.py move_Work"
# CMD = "ssh palletizer-0 -l er python3 btr_myPalletizer.py move_Work"
CMD = "ssh palletizer-0 -l er -i /home/robotino/.ssh/id_rsa_pall python3 /home/er/yasuda_pall/eindhoven2024/MyPallBTR.py"

def grab_Arm(data):
    cmd = CMD + " moveG"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def release_Arm(data):
    cmd = CMD + " moveR"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node("BTR_myCobot_ros")
    src00 = rospy.Service('/btr/move_g', Empty, grab_Arm)
    src01 = rospy.Service('/btr/move_r', Empty, release_Arm)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

    print("Bye...")
