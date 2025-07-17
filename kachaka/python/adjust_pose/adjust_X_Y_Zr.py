from kachaka_api import KachakaApiClient
import math

class adjust_X_Y_Zr():
    def __init__(self, client):
        self.client = client
        current_pose = self.client.get_robot_pose()
        print(f"current pose: {current_pose}")

    def adjust_X(self, x, speed=0.1):
        x = -x #
        self.client.move_forward(x, speed=speed)

    def adjust_Y(self, y):
        # move shift_x m for adjusting y direction.
        shift_x = 0.3
        if y == 0:
            pass

        elif abs(y) <= 0.3:
            y = -y #
            turn_radian = math.atan2(y, shift_x)
            print(turn_radian)
            self.adjust_X(shift_x)
            self.adjust_Zr(-turn_radian)
            self.adjust_X(-(shift_x/math.cos(turn_radian)))
            self.adjust_Zr(turn_radian)

        else:
            y = -y #
            self.adjust_Zr((math.pi/2)*(y/abs(y)))
            self.adjust_X(y*(y/abs(y)))
            self.adjust_Zr(-(math.pi/2)*(y/abs(y)))
            

    def adjust_Zr(self, r_z):
        self.client.rotate_in_place(r_z)

def main():
    client = KachakaApiClient(target="10.42.10.201:26400")
    adjustor = adjust_X_Y_Zr(client)
    adjustor.adjust_X(0.5, speed=0.3)
    # adjustor.adjust_Y(0.31)
    adjustor.adjust_Zr(math.pi/3)
    adjustor.adjust_Zr(-math.pi/3)
    
if __name__ == "__main__":
    main()
