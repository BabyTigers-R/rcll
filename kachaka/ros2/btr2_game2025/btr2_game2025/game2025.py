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

    challengeFlag = True
    oldGamePhase = -1
    oldGameState = -1

    while True:
        while rclpy.ok():
            rclpy.spin_once(refbox)
            '''
            if refbox.future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（>）がTrueになる。
                try:
                    response = refbox.future.result() # サーバーから非同期的に送られてきたレスポ>ンス
                except Exception as e:                                         #  エラー時の処理
                    refbox.get_logger().info(
                        'Service call failed %r' % (e,))
                else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                    refbox.get_logger().info('Response:=%s' % response)
                break
            '''
            ### check for receiving GameState information
            if (refbox.refboxGameStateFlag == True):
                ### Check for changing of GameState
                if (oldGameState != refbox.refboxGameState):
                    print("game2025:", challenge, refbox.refboxGameState)
                    oldGameState = refbox.refboxGameState
                if (challengeFlag):
                    ### Check for changing of GmaePhase
                    if (oldGamePhase != refbox.refboxGamePhase):
                        print("refboxGamePhase: ", refbox.refboxGamePhase)
                        oldGamePhase = refbox.refboxGamePhase
                    

                    ### For Challenge Track
                    
                    #### Exploration Challenge
                    if (challenge == "exploration" and refbox.refboxGamePhase == 20 ):
                        rcll.challenge("exploration")
                        challengeFlag = False
                    
                    #### Production Challenge
                    elif (challenge == "production" and refbox.refboxGamePhase == 30):
                        rcll.challenge("production")
                        challengeFlag = False

                    #### Grasping Challenge
                    elif (challenge == "grasping" and refbox.refboxGamePhase == 30):
                        rcll.challenge("grasping")
                        challengeFlag = False

                    #### Navigation Challenge
                    elif (challenge == "navigation" and refbox.refboxGamePhase == 30):
                        rcll.challenge("navigation")
                        challengeFlag = False

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

