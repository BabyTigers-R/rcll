import os
import sys
sys.path.append("../../../")

from kachaka_api import KachakaApiClient
import math
from kachaka.python.adjust_pose.adjust_X_Y_Zr import adjust_X_Y_Zr
from kachaka.python.navigation.controller import move_kachaka_to_pose

from python.module_photographer import module_photographer
from python.module_belt_detect import module_belt_detect
from python.module_c0_detect import module_c0_detect

def startGrasping():
    
    pg = module_photographer()
    bd = module_belt_detect()

    grasping_position = [0.20, 0.0, 0]

    print("#==================#")
    print("start grasping")
    print("------")
    
    # make kachaka api client
    client = KachakaApiClient(target="10.42.10.201:26400")

    # a class for adjusting kachaka pose
    x_y_zr_adjuster = adjust_X_Y_Zr(client)

    # move to the place
    #move_kachaka_to_pose(1.15, 0.0, -math.pi/2, client)
    client.move_to_pose(1.20, -2.0, -math.pi/2)

    # detect belt position
    belt_position = [0.25, 0.1, math.pi/12]
    
    position_error = []
    for i in range(len(grasping_position)):
        position_error.append(grasping_position[i] - belt_position[i])

    print("distance between kachaka and grasping position: ", position_error)
    # adjust x direction error 
    #x_y_zr_adjuster.adjust_X(-position_error[0])
    # adjust y direction error 
    #x_y_zr_adjuster.adjust_Y(-position_error[1])
    x_y_zr_adjuster.adjust_Y(0.05)
    # adjust z axis rotation error 
    #x_y_zr_adjuster.adjust_Zr(-position_error[2])

def main():
    startGrasping()

if __name__ == "__main__":
    main()
