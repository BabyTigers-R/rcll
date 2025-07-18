#!/usr/bin/python
import sys
import subprocess
import rclpy
import btr2_rcll2025
import btr2_refbox

def main(args=None):
    rclpy.init(args=args)
    args = sys.argv
    topicName = ""
    gazeboFlag = True
    robotNum = 1

    ### set challenge information by args
    if (len(args) >= 2):
        challenge = args[1]
        if (len(args) >= 3):
            robotNum = int(args[2])
        if (gazeboFlag == True):
            topicName = "/robotino" + str(robotNum)

    ### setup for RefBox
    refbox = btr2_refbox.refbox(teamName = "Babytigers-R", robotNum = robotNum, gazeboFlag = False)
    refbox.sendBeacon()

    ### setup for rcll
    rcll   = btr2_rcll2025.btr2_rcll(teamName = "Babytigers-R", robotNum = robotNum, gazeboFlag = False, refbox = refbox)

    challenge_flag = True
    old_game_phase = -1
    old_game_state = -1

    while True:
        while rclpy.ok():
            rclpy.spin_once(refbox)
            ### check for receiving GameState information
            if (refbox.refboxGameStateFlag == True):
                ### Check for changing of GameState
                if (old_game_state != refbox.refboxGameState):
                    print("game2025:", challenge, refbox.refboxGameState)
                    old_game_state = refbox.refboxGameState
                if (challenge_flag):
                    ### Check for changing of GmaePhase
                    if (old_game_phase != refbox.refboxGamePhase):
                        print("refboxGamePhase: ", refbox.refboxGamePhase)
                        old_game_phase = refbox.refboxGamePhase
                    

                    ### For Challenge Track
                    
                    #### Exploration Challenge
                    if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
                        rcll.challenge("exploration")
                        challenge_flag = False
                    
                    #### Production Challenge
                    elif (challenge == "production" and refbox.refboxGamePhase == 30):
                        rcll.challenge("production")
                        challenge_flag = False

                    #### Grasping Challenge
                    elif (challenge == "grasping" and refbox.refboxGamePhase == 30):
                        rcll.challenge("grasping")
                        challenge_flag = False

                    #### Navigation Challenge
                    elif (challenge == "navigation" and refbox.refboxGamePhase == 30):
                        rcll.challenge("navigation")
                        challenge_flag = False

                    ### for Main Track
                    elif (challenge == "main"):
                        refbox.sendBeacon()
                        print("Game status is ", refbox.refboxGamePhase)
                        if (refbox.refboxGamePhase == 20):
                            rcll.challenge("main_exploration")
                        if (refbox.refboxGamePhase == 30):
                            rcll.challenge("main_production")

# main
#
if __name__ == '__main__':
    main()

