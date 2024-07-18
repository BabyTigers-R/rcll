#!/usr/bin/python
import sys
import subprocess
import rospy
import btr_refbox
import btr_rcll2024

# main
#
if __name__ == '__main__':
    args = sys.argv
    topicName = ""
    gazeboFlag = True
    robotNum = 1
    #
    # check for Robotino CPU
    # 
    # "model name	: Intel(R) Core(TM) i5 CPU       E 520  @ 2.40GHz"
    command = "cat /proc/cpuinfo"
    all_info = subprocess.check_output(command, shell=True).decode().strip()
    for line in all_info.split("\n"):
        if "model name" in line:
            if "E 520  @ 2.40GHz" in line:
                gazeboFlag = False
    print(gazeboFlag)

    if (len(args) >= 2):
        challenge = args[1]
        if (len(args) >= 3):
            robotNum = int(args[2])
        if (gazeboFlag == True):
            topicName = "/robotino" + str(robotNum)

    nodeName = "btr_2024_" + str(robotNum)
    print("Node name:" + nodeName)
    rospy.init_node(nodeName)
    rate = rospy.Rate(10)

    refbox = btr_refbox.refbox(teamName = "BabyTigers-R", robotNum = robotNum, gazeboFlag = gazeboFlag)
    rcll   = btr_rcll2024.btr_rcll(teamName = "BabyTigers-R", robotNum = robotNum, gazeboFlag = gazeboFlag, refbox = refbox)

    # rcll.init(challenge)

    print(challenge)
    challengeFlag = True
    # initField()
    # while True:
    oldGamePhase = -1
    while not rospy.is_shutdown():
    
        print("game2024: ", challenge, refbox.refboxGamePhase)
        if (challengeFlag):
            if (oldGamePhase != refbox.refboxGamePhase):
                print("refboxGamePhase: ", refbox.refboxGamePhase)
                oldGamePhase = refbox.refboxGamePhase
            if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
                rcll.challenge("exploration")
                challengeFlag = False
            elif (challenge == "grasping" and refbox.refboxGamePhase == 30):
            # elif (challenge == "grasping"):
                print(refbox.refboxGamePhase)
                rcll.challenge("grasping")
                challengeFlag = False
            elif (challenge == "graspingTest"):
                print(refbox.refboxGamePhase)
                rcll.challenge("graspingTest")
                challengeFlag = False
            elif (challenge == "navigation" and refbox.refboxGamePhase == 30):
                rcll.challenge("navigation")
                challengeFlag = False
            elif (challenge == "machineTest" and refbox.refboxGamePhase == 30):
                rcll.challenge("prepareMachineTest")
                challengeFlag = False
            elif (challenge == "production" and refbox.refboxGamePhase == 30):
                rcll.challenge("production")
                challengeFlag = False
            elif (challenge == "beacon"):
                refbox.sendBeacon()
                print("Game status is ", refbox.refboxGamePhase)
            elif (challenge == "clockwise"):
                rcll.challenge("clockwise")
            if (challenge == "gazebo"):
                print("Game status is ", refbox.refboxGamePhase)
                rcll.challenge("turn")
                challengeFlag = False
                
                if (refbox.refboxGamePhase == 10):
                    rcll.challenge("reset1")
                if (refbox.refboxGamePhase == 20):
                    rcll.challenge("main_exploration")
                if (refbox.refboxGamePhase == 30):
                    rcll.challenge("production")
        # print(refbox.refboxGamePhase)

        refbox.sendBeacon()
        rate.sleep()


