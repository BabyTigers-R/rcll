import math

class IkSolver():
    def __init__(self):
        # length of end link
        self.l_end = 30
        self.l_end_dash = 34.12
        # parameters of myPalletizer
        self.l_base_theta2 = 125
        self.l_theta2_theta3 = 130

        self.end_angular_bias = 80

    def __call__(self, x, y, z, omega):
        theta1_dash = math.atan2(x, -z)
        theta1 = theta1_dash - math.pi/2
        theta4 = math.radians(omega) + theta1

        middle_pos = (x - self.l_end_dash*math.sin(theta1_dash), y - self.l_end, z + self.l_end_dash*math.cos(theta1_dash))
        d_theta2_middle = math.sqrt((middle_pos[0])**2 + (middle_pos[1] + self.l_base_theta2)**2 + (middle_pos[2])**2)
        
        if (2*self.l_theta2_theta3) <  d_theta2_middle:
            return None, None, None, None
        
        theta3 = math.asin((2*self.l_theta2_theta3 - d_theta2_middle) / (4*self.l_theta2_theta3))

        theta2 = math.acos((-middle_pos[1] - self.l_base_theta2) / d_theta2_middle) - ((math.pi/2 + theta3) / 2)

        # convert radian to degree
        theta1 = math.degrees(theta1)
        theta2 = math.degrees(theta2)
        theta3 = math.degrees(theta3)
        theta4 = math.degrees(theta4) - self.end_angular_bias

        return theta1, theta2, theta3, theta4
        
def main():
    x, y, z, omega = 200, 50, 0, 0
    #x, y, z, omega = 100, 50, 0, math.pi/2
    IKS = IkSolver()
    theta1, theta2, theta3, theta4 = IKS(x, y, z, omega)
    print(theta1, theta2, theta3, theta4)

if __name__ == "__main__":
    main()
