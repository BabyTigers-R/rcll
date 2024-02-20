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

    nodeName = "btr2024_" + str(robotNum)
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
    
        if (challenge == "nbr33" and challengeFlag):
            rcll.challenge("nbr33")
            challengeFlag = False

        if (challenge == "exploration" and challengeFlag and refbox.refboxGamePhase == 20 ):
            rcll.challenge("exploration")
            challengeFlag = False

        if (challenge == "gripping" and challengeFlag):
            rcll.challenge("gripping")
            challengeFlag = False

        if (challenge == "graspingTest" and challengeFlag):
            rcll.challenge("graspingText")
            challengeFlag = False

        if (challenge == "driving" and challengeFlag):
            rcll.challenge("driving")
            challengeFlag = False

        if (challenge == "positioning" and challengeFlag):
            rcll.challenge("positioning")
            challengeFlag = False

        if (refbox.refboxGamePhase == 30 and challenge == "grasping" and challengeFlag):
            rcll.challenge("grasping")
            challengeFlag = False

        if (challenge == "navigationTest" and challengeFlag):
            rcll.challenge("navigationTest")
            challengeFlag = False

        if (refbox.refboxGamePhase == 30 and challenge == "navigation" and challengeFlag):
            if (refbox.refboxMachineInfoFlag and refbox.refboxNavigationRoutesFlag):
                rcll.challenge("navigation")
                challengeFlag = False

        # send machine prepare command
        if (refbox.refboxGamePhase == 30 and challenge == "" ):
            rcll.challenge("prepareMachineTest")
            challengeFlag = False

        if ( challenge == "test" and challengeFlag):
            rcll.challenge("test")
            challengeFlag = False

        if (challenge == "test_by_c920"):
            rcll.challenge("c920")
            challengeFlag = False

        if (challenge == "test_C0"):
            rcll.challenge("C0")
            challengeFlag = False
    
        if (challenge == "testOpen"):
            rcll.challenge("testOpen")
            challengeFlag = False
    
        if (challenge == "beacon"):
            refbox.sendBeacon()
            print("Game status is ", refbox.refboxGamePhase)

        if (challenge == "clockwise" and challengeFlag):
            rcll.challenge("turnClockwise")
            challengeFlag = False

        if (challenge == "camera" and challengeFlag):
            rcll.challenge("camera")
            challengeFlag = False

        if (challenge == "gazebo"):
            refbox.sendBeacon()
            print("Game status is ", refbox.refboxGamePhase)

        refbox.sendBeacon()
        rate.sleep()


