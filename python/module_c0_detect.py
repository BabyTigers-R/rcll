import numpy as np
import cv2
import sys
import time
from module_photographer import module_photographer

class module_c0_detect():
    def __init__(self):
        self.tolerance = 7

    def __call__(self, img):
        self.img = img
        self.result = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        self.w = self.img.shape[1]
        self.h = self.img.shape[0]
        self.ref_line = int(self.w/2) #- 88

        #
        ## calculate center of gravity
        #
        ret, thresh = cv2.threshold(self.img, 127, 255, 0)
        img1 = np.array(thresh)

        contours, _ = cv2.findContours(img1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        center_list = []
        for c in contours:
            try:
                M = cv2.moments(c)
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                #cv2.circle(self.result, (x, y), 5, (0, 255, 0), thickness=3)
                center_list.append([x, y])
            except:
                return 2

        #
        ##
        #

        #
        ## draw circle markers
        #
        if len(center_list) < 1:
            cv2.putText(self.result, text='No work detect!', org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_4)
            self.show_result()
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

            # center
            if abs(minimum_error) <= self.tolerance:
                cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 255, 0))
                for l in range(len(center_list)):
                    if l == minimum_error_index:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (0, 255, 0), 3)
                    else:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (0, 0, 255), 3)

                self.show_result()
                return 0

            # left or right
            else:
                cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (255, 0, 0))
                for l in range(len(center_list)):
                    if l == minimum_error_index:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (255, 0, 0), 3)
                    else:
                        cv2.circle(self.result, (center_list[l][0], center_list[l][1]), 5, (0, 0, 255), 3)
                
                self.show_result()
                return int(-(minimum_error / abs(minimum_error)))
        #
        ##
        #

        # return sorted(center_list, key=lambda x:abs(x[0]-self.w/2))

    def show_result(self):
        cv2.imwrite('./images/c0.png', self.result)

        #cv2.imshow('res', self.res)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

if __name__ == "__main__":
    pg = module_photographer()
    time.sleep(3)
    if len(sys.argv) > 1:
        name = sys.argv[1]
        gray, bg_removed = pg(name)
    else:
        gray, bg_removed = pg()
    pg.pipeline.stop()

    c0d = module_c0_detect()
    error = c0d(bg_removed)
    print(error)
    #result = inst()
    #print(result)
    #inst.show_result()
