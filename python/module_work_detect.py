# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import sys

class module_work_detect():
    def __init__(self, img):
        self.img = img
        self.w, self.h = self.img.shape[::-1]
        # reference height
        self.ref = int(self.h/2) + 30
        # width to be regarded as work
        self.work_w = int(50)
        self.work_l = 0
        self.work_r = self.w
        self.tolerance = 10 # pixel
        
    def detect(self):
        self.result = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        # draw center line
        cv2.line(self.result, (int(self.w/2), 0), (int(self.w/2), int(self.h)), (0, 255, 0))
        # draw reference line
        cv2.line(self.result, (0, self.ref), (int(self.w), self.ref), (255, 0, 0))
        res, self.img_bin = cv2.threshold(self.img, 1, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        self.img_bin = cv2.morphologyEx(self.img_bin, cv2.MORPH_CLOSE, kernel)
        #cv2.imshow("img_bin", self.img_bin)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        self.img_bin = (self.img_bin/255).astype(np.uint8)

        sample = self.img_bin[self.ref]
        continuous = self.judge(sample)
        # (The sumation of pixel in the binary img have a value of 1 for the referenced height) >= (width of work)
        if np.sum(continuous) >= self.work_w:
            self.work_l = int(np.where(continuous==1)[0][0])
            self.work_r = int(np.where(continuous==1)[0][-1])
            self.average = int((self.work_l + self.work_r)/2)
            cv2.line(self.result, (self.work_l, 0), (self.work_l, self.h), (0, 0, 255))
            cv2.line(self.result, (self.work_r, 0), (self.work_r, self.h), (0, 0, 255))
            # output position error (right or left)
            if (int(self.w/2) - self.tolerance) <= self.average and self.average <= (int(self.w/2) + self.tolerance):
                cv2.circle(self.result, (self.average, self.ref), 5, (0, 255 , 0), thickness=3)
                print("Tolerable, go.")
                return 0
            elif self.average < (int(self.w/2) - self.tolerance):
                cv2.circle(self.result, (self.average, self.ref), 5, (0, 0 , 255), thickness=3)
                print("Left")
                return 1
            elif self.average > (int(self.w/2) + self.tolerance):
                cv2.circle(self.result, (self.average, self.ref), 5, (0, 0 , 255), thickness=3)
                print("Right")
                return -1
        else:
            print("No work detected!")
            return 2

    def judge(self, sample):
        continuous = self.check_continuous(sample, self.work_w)
        return continuous

    # Function to return how many pixels in the binary img have a value of 1 for the referenced height
    def is_continuous(self, arr, left_cnt):
        if left_cnt >= 1:
            #arr2 = arr[:, :-1] & arr[:, 1:]
            arr2 = arr[:-1] & arr[1:]
            arr2 = self.is_continuous(arr2, left_cnt - 1)
        else:
            arr2 = arr
        return arr2

    def check_continuous(self, arr, continuous_cnt):
        arr2 = self.is_continuous(arr, continuous_cnt - 1)
        #arr3 = np.tile(np.full(continuous_cnt - 1, 0), arr2.shape[0]).reshape(-1, arr2.shape[0]).T
        arr3 = (np.full(continuous_cnt - 1, 0))
        arr2 = np.hstack([arr2, arr3])
        for _ in range(continuous_cnt - 1):
            #arr2[:, 1:] = arr2[:, 1:] | arr2[:, :-1]
            arr2[1:] = arr2[1:] | arr2[:-1]
        print(sum(arr2))
        return arr2
    #/ Function to return how many pixels in the binary img have a value of 1 for the referenced height

    def show_result(self):
        #cv2.imshow("result", self.result)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        return self.result

def main():
    #jpg画像の名前
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        print("Error! Please set args!!!")
        quit()

    # read image
    img = cv2.imread("{}.jpg".format(name), 0)

    inst = module_work_detect(img)
    action_to_take = inst.detect()
    print(action_to_take)
    result = inst.show_result()
    cv2.imwrite("{}_a.png".format(name), result)

if __name__ == "__main__":
    main()

