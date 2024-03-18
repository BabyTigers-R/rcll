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
    
        if (challengeFlag):
            if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
                rcll.challenge("exploration")
            elif (challenge == "grasping" and refbox.refboxGamePhase == 30):
                rcll.challenge("grasping")
            elif (challenge == "navigation" and refbox.refboxGamePhase == 30):
                rcll.challenge("navigation")
            elif (challenge == "machineTest" and refbox.refboxGamePhase == 30):
                rcll.challenge("prepareMachineTest")
            elif (challenge == "beacon"):
                refbox.sendBeacon()
                print("Game status is ", refbox.refboxGamePhase)
            elif (challenge != "gazebo"):
                ### for challenge Track
                # nbr33, gripping, graspingTest, driving
                # positioning 
                #
                ### for test at challenge Track
                # navigationTest
                # 
                ### for gripper
                # c920, C0, testOpen
                # 
                ### for test
                # test, turnClockwise, camera
                rcll.challenge(challenge)

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
            else:
                challengeFlag = False

        refbox.sendBeacon()
        rate.sleep()


