import numpy as np
import cv2
import sys

class module_center_of_gravity_detect():
    def __init__(self, name, gazebo=False):
        self.gazebo = gazebo
        self.name = name
        self.img = cv2.imread("./images/{}.jpg".format(self.name), 0)
        #print(self.templ.shape)
        #print(self.img.shape)
        #gray = cv2.imread("{}_gray.jpg".format(self.name), 0)
        self.result = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        self.w = self.img.shape[1]
        self.h = self.img.shape[0]
        self.tolerance = 7
        self.ref_line = int(self.w/2)

    def run(self):
        if self.gazebo:
            return 0
        #
        ## calculate center of gravity
        #
        ret, thresh = cv2.threshold(self.img, 127, 255, 0)
        img1 = np.array(thresh)

        contours, _ = cv2.findContours(img1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        center_list = []
        for c in contours:
            M = cv2.moments(c)
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            #cv2.circle(self.result, (x, y), 5, (0, 255, 0), thickness=3)
            center_list.append([x, y])

        #
        ##
        #

        #
        ## draw circle markers
        #
        if len(center_list) < 1:
            cv2.putText(self.result, text='No work detect!', org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_4)
            return 2
        
        else:
            minimum_error = self.w
            minimum_error_index = 0
            for l in range(len(center_list)):
                if abs(minimum_error) > abs(center_list[l][0] - self.ref_line):
                    minimum_error = center_list[l][0] - self.ref_line
                    minimum_error_index = l
                else:
                    pass

            if abs(minimum_error) <= self.tolerance:
                cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 255, 0))
                for l in range(len(center_list)):
                    if l == minimum_error_index:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (0, 255, 0), 3)
                    else:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (0, 0, 255), 3)

                return 0

            else:
                cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (255, 0, 0))
                for l in range(len(center_list)):
                    if l == minimum_error_index:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (255, 0, 0), 3)
                    else:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (0, 0, 255), 3)
                
                return int(-(minimum_error / abs(minimum_error)))
        #
        ##
        #

    def show_result(self):
        # 画像の保存
        cv2.imwrite('./images/{}_result.png'.format(self.name), self.result)

        #cv2.imshow('res', self.res)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

if __name__ == "__main__":
    #jpg画像の名前
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        print("Error! Please set args!!!")
        quit()

    inst = module_center_of_gravity_detect(name)
    result = inst.run()
    print(result)
    inst.show_result()
