#!/usr/bin/python
import sys
import rospy
import btr_refbox
import btr_rcll2024

# main
#
if __name__ == '__main__':
    args = sys.argv
    topicName = ""
    gazeboFlag = False
    robotNum = 1
    if (len(args) >= 2):
        challenge = args[1]
        if (len(args) >= 3):
            robotNum = int(args[2])
        if (challenge == "gazebo" or challenge == "gazebo1"):
            topicName = "/robotino" + str(robotNum)
            gazeboFlag = True

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
    while not rospy.is_shutdown():
    
        print("game2024: ", challenge, refbox.refboxGamePhase)
        if (challengeFlag):
            if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
                rcll.challenge("exploration")
                challengeFlag = False
            #elif (challenge == "grasping" and refbox.refboxGamePhase == 30):
            elif (challenge == "grasping"):
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

        refbox.sendBeacon()
        rate.sleep()


