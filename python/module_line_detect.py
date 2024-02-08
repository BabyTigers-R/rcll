# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import sys

class module_line_detect():
    def __init__(self, img):
        #ほぼ縦線のみを抽出するときの閾値g
        self.g = 1
        #それぞれの線分の式の傾き、切片、線分の始点終点のリストを格納するためのリスト
        self.alist = []
        self.blist = []
        self.rlines = []


        #直線を連結するために、傾き、切片がほぼ同じものを抽出するときの閾値
        self.alpha = 0.01
        self.beta = 1.0
        #基準線分ともう一つの線分のyの値を格納するリスト
        self.ymm = []
        #連結するかどうかのフラグ
        self.chain_flag = 0
        #連結した線分の始点終点を一時的に格納するリストとそれを格納するリスト
        self.t = []
        self.tlines = []
        #線分の長さで抽出する直線を変えるための閾値
        self.length = 40

        #線分の式との交点を出すときの式y =ycとそのときの交点のx座標を格納するためのリスト、各x座標の合計
        self.img = img
        self.img_w = int(self.img.shape[1])
        self.img_h = int(self.img.shape[0])

        self.yc = int(self.img_h*(3/4))
        self.xc1 = self.img_w - 120
        self.xc2 = self.img_w + 120

        #yc = 320
        #xc1 = 170
        #xc2 = 410
        self.xlists = []
        self.xtotal = 0

        #xlistsの平均xcenterとxlistの最大最小値の平均xa
        self.xcenter = -1
        self.xa = -1
        #アームで取りにいってもいいという閾値arm_min,arm_max
        self.arm_min = (self.img_w/2) - 15
        self.arm_max = (self.img_w/2) + 15

    def out1(self):
        #FLT(FastLineDetector)の設定
        length_threshold = 20
        distance_threshold = 1.41421356
        canny_th1 = 100
        canny_th2 = 100
        canny_aperture_size = 3
        do_merge = False

        self.fld = cv2.ximgproc.createFastLineDetector(
            length_threshold,
            distance_threshold,
            canny_th1,
            canny_th2,
            canny_aperture_size,
            do_merge
        )

        #FLTを行って、線分の始点終点を格納([[[始点のx 始点のy 終点のx 終点のy)], ・・・ ,[]]]
        self.lines = self.fld.detect(self.img)
        #FLTの出力した線分を画像に反映
        self.out1 = self.fld.drawSegments(self.img, self.lines)

    def out2(self):
        #次元を1つ減らす
        self.lineS = np.squeeze(self.lines)
        #print(len(lineS))
        #print(lineS)
        #FLTの出力した線分があるのなら、各始点終点の直線の傾き、切片を求め、傾きがgの範囲内なら、傾き、切片、線分の始点終点を格納
        ##画像のx軸を実軸のy軸、画像のy軸を実軸のx軸として計算
        ###gは0付近の値にする必要がある
        if len(self.lineS) != 0:
            for i in range(len(self.lineS)):
                if (self.lineS[i][3]-self.lineS[i][1]) == 0:
                    continue
                a = (self.lineS[i][2]-self.lineS[i][0])/(self.lineS[i][3]-self.lineS[i][1])
                if a >= -self.g  and a <= self.g:
                    self.alist.append(a)
                    self.blist.append(self.lineS[i][2]-(a*self.lineS[i][3]))
                    self.rlines.append(self.lineS[i])
        #print(alist)
        #print(blist)
        #次元を1つ減らす
        self.rlines = np.squeeze(self.rlines)
        #print(len(rlines))
        #print(rlines)
        #ほぼ縦線のみの線分を画像に反映
        self.out2 = self.fld.drawSegments(self.img, self.rlines)

    def out3(self):
        #ほぼ縦線のみの線分があるのなら、先程求めた傾き、切片がほぼ等しい線分を連結
        ##ワークなどでベルトコンベアが遮られているかもしれないため
        ###今は基準となる線分と他の線分で比べるだけで、他の線分が2つ以上出てきたと>きの処理をしていない
        ####基準線分と連結できない場合は基準線分を格納、連結できる場合は連結したも>のを格納
        ####線分の長さlengthによって格納するかどうかを決める↑
        if len(self.rlines) != 0:
            for i in range(len(self.rlines)-1):
                for j in range(i+1,len(self.rlines)):
                    #傾きはalpha内、切片はbeta内にあるときは直線を連結しに行く
                    if self.alist[i] >= self.alist[j]-self.alpha and self.alist[i] <= self.alist[j]+self.alpha and self.blist[i] >= self.blist[j]-self.beta and self.blist[i] <= self.blist[j]+self.beta:
                        #print(rlines[i],rlines[j])
                        #print(alist[i],alist[j])
                        #print(blist[i],blist[j])
                        #基準線分と今見ている線分の各始点終点のy座標を格納し、その>中の最大値、最小値を出す
                        self.ymm =[self.rlines[i][1],self.rlines[i][3],self.rlines[j][1],self.rlines[j][3]]
                        ymax = self.ymm.index(max(self.ymm))
                        ymin = self.ymm.index(min(self.ymm))
                        #print(ymm,ymin,ymax)
                        if ymin == 0:
                            if ymax == 1:
                                self.t = [self.rlines[i][0],self.rlines[i][1],self.rlines[i][2],self.rlines[i][3]]
                            if ymax == 2:
                                self.t = [self.rlines[i][0],self.rlines[i][1],self.rlines[j][0],self.rlines[j][1]]
                            else:
                                self.t = [self.rlines[i][0],self.rlines[i][1],self.rlines[j][2],self.rlines[j][3]]
                        if ymin == 1:
                            #print("ymin1")
                            if ymax == 0:
                                self.t = [self.rlines[i][2],self.rlines[i][3],self.rlines[i][0],self.rlines[i][1]]
                            if ymax == 2:
                                self.t = [self.rlines[i][2],self.rlines[i][3],self.rlines[j][0],self.rlines[j][1]]
                            else:
                                self.t = [self.rlines[i][2],self.rlines[i][3],self.rlines[j][2],self.rlines[j][3]]
                        if ymin == 2:
                            #print("ymin2")
                            if ymax == 0:
                                self.t = [self.rlines[j][0],self.rlines[j][1],self.rlines[i][0],self.rlines[i][1]]
                            if ymax == 1:
                                self.t = [self.rlines[j][0],self.rlines[j][1],self.rlines[i][2],self.rlines[i][3]]
                            else:
                                self.t = [self.rlines[j][0],self.rlines[j][1],self.rlines[j][2],self.rlines[j][3]]
                        if ymin == 3:
                            #print("ymin3")
                            if ymax == 0:
                                self.t = [self.rlines[j][2],self.rlines[j][3],self.rlines[i][0],self.rlines[i][1]]
                            if ymax == 1:
                                self.t = [self.rlines[j][2],self.rlines[j][3],self.rlines[i][2],self.rlines[i][3]]
                            else:
                                self.t = [self.rlines[j][2],self.rlines[j][3],self.rlines[j][0],self.rlines[j][1]]
                        self.chain_flag = 1
                if self.chain_flag == 1:
                    #print("line chain")
                    self.chain_flag = 0
                else:
                    self.t = [self.rlines[i][0],self.rlines[i][1],self.rlines[i][2],self.rlines[i][3]]
                #lengthによって、長さの小さい線分を格納しないようにする
                line_length = math.sqrt((self.t[2] - self.t[0])**2 + (self.t[3] - self.t[1])**2)
                if line_length >= self.length and self.t[1] <= self.yc and self.t[3] >= self.yc :
                    #print(t)
                    #print("\n")
                    self.tlines.append(self.t)
        #次元を1つ減らす
        #If tlines has component less than two, program exit.
        if len(self.tlines) <= 1:
            print("Dimensions cannot be reduced, because tlines list has only one component!!!")
            return 3
        self.tlines = np.squeeze(self.tlines)
        #print(len(tlines))
        #print(tlines)
        #print(type(tlines))
        #ほぼ縦線を連結できるものは連結し、線分の長さがlengthより大きい線分を画像に反映
        self.out3 = self.fld.drawSegments(self.img, self.tlines)
        return 0

    def out4(self):
        #Make a line segment into a straight line.
        self.alist.clear()
        self.blist.clear()
        self.rlines = []
        if len(self.tlines) != 0:
            for i in range(len(self.tlines)):
                if (self.tlines[i][2]-self.tlines[i][0]) == 0:
                    continue
                a = (self.tlines[i][3]-self.tlines[i][1])/(self.tlines[i][2]-self.tlines[i][0])
                b = int(self.tlines[i][3]-(a*self.tlines[i][2]))
                y = int(a*self.img_w + b)
                self.alist.append(a)
                self.blist.append(b)
                self.rlines.append([0, b, self.img_w, y])

        #Find cross points of straight lines in rlines
        #print(self.alist)
        #print(self.blist)
        #print(self.rlines)
        self.tlines = []
        if len(self.rlines) > 1:
            for i in range(len(self.rlines)):
                for j in range(len(self.rlines)):
                    if (self.alist[i]-self.alist[j]) == 0:
                        continue
                    x_x = int((self.blist[j]-self.blist[i])/(self.alist[i]-self.alist[j]))
                    y_x = int(self.alist[i]*x_x + self.blist[i])
                    #If the cross point is in center of img frame, storage the straight line.
                    if x_x >= int(self.img_w*(float(1/3))) and x_x <= int(self.img_w*(float(2/3))) and y_x >= int(self.img_h*(float(1/3))) and y_x <= int(self.img_h*(float(2/3))):
                        self.tlines.append(self.rlines[i])

        self.rlines = self.tlines
        #print(self.rlines)
        self.tlines = np.array(self.tlines)
        #次元を1つ減らす
        #If tlines has component less than two, program exit.
        if len(self.tlines) <= 1:
            print("Dimensions cannot be reduced, because tlines list has only one component!!!")
            return 3
        self.tlines = np.squeeze(self.tlines)

        #draw straight lines and img center
        self.out4 = cv2.rectangle(self.img, (int(self.img_w*(float(1/3))), int(self.img_h*(float(1/3)))), (int(self.img_w*(float(2/3))), int(self.img_h*(float(2/3)))), (0, 0, 0))
        for i in self.rlines:
            cv2.line(self.out4, (i[0], i[1]), (i[2], i[3]), (0, 0, 255), thickness=1)
        return 0

    def detect(self):
        self.out1()
        self.out2()
        out3 = self.out3()
        if out3 == 3:
            print("robot did not detect belt conveyor at out3")
            return 2
        out4 = self.out4()
        if out4 == 3:
            print("robot did not detect belt conveyor at out4")
            return 2

        if len(self.tlines) != 0:
            for i in range(len(self.tlines)):
                if (self.tlines[i][3]-self.tlines[i][1]) == 0:
                    continue
                aa = (self.tlines[i][2]-self.tlines[i][0]) / (self.tlines[i][3]-self.tlines[i][1])
                bb = (self.tlines[i][2] - (aa * self.tlines[i][3]))
                xx = int(aa * self.yc + bb)
                print(xx)
                #if xx >= xc1 and xx <= xc2:
                self.xlists.append(xx)
                cv2.circle(self.out4, (xx,self.yc), 5, (255,0,0), thickness=0, lineType=cv2.LINE_AA)
                self.xtotal += xx
            print(self.xlists)
            self.xcenter = int((self.xtotal/len(self.tlines)))
            cv2.circle(self.out4, (self.xcenter,self.yc), 5, (0,255,0), thickness=0, lineType=cv2.LINE_AA)
        if self.xcenter != -1:
            if self.xcenter >= self.arm_min and self.xcenter <= self.arm_max:
                print("go")
                return 0
            elif self.xcenter <= self.arm_min:
                print("left")
                return 1
            else:
                print("right")
                return -1
        else:
            print("robot did not detect belt conveyor")
            return 2

    def show_result(self, name):
        cv2.imwrite("{}a.png".format(name),self.out1)
        cv2.imwrite("{}b.png".format(name),self.out2)
        cv2.imwrite("{}c.png".format(name),self.out3)
        cv2.imwrite("{}d.png".format(name),self.out4)


def main():
    #jpg画像の名前
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        print("Error! Please set args!!!")
        quit()

    # read image
    img = cv2.imread("{}.jpg".format(name), 0)

    inst = module_line_detect(img)
    result = inst.detect()
    print(result)
    if result == 2:
        return 0
    inst.show_result(name)

if __name__ == "__main__":
    main()

