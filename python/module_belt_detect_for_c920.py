import numpy as np
import cv2
import sys

class module_belt_detect_for_c920():
    def __init__(self, name, gazebo=False):
        self.gazebo = gazebo
        self.name = name
        self.templ = cv2.imread("./templ/template_for_c920.png", 0)
        self.img = cv2.imread("./images/{}.jpg".format(self.name), 0)
        #print(self.templ.shape)
        #print(self.img.shape)
        #gray = cv2.imread("{}_gray.jpg".format(self.name), 0)
        self.result = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        self.w = self.img.shape[1]
        self.h = self.img.shape[0]
        self.tolerance = 5
        self.ref_line = int(self.w/2)

    def run(self):
        if self.gazebo:
            return 0
        # 処理対象画像に対して、テンプレート画像との類似度を算出する
        self.res = cv2.matchTemplate(self.img, self.templ, cv2.TM_CCOEFF_NORMED)
        #cv2.imshow(" ", self.res)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #self.res = cv2.GaussianBlur(self.res, (151, 151), 0.01)
        #cv2.imshow(" ", self.res)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        # 類似度の高い部分を検出する
        threshold = 0.5
        matching_rate = np.max(self.res)
        #print(res.dtype)
        loc = np.where(self.res == matching_rate)
        #print(loc)
        matching_loc = np.array([loc[1][0], loc[0][0]])
        print(matching_rate)

        if matching_rate < threshold:
            cv2.putText(self.result, text='No matching location', org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_4)
            return 2

        # テンプレートマッチング画像の高さ、幅を取得する
        h, w = self.templ.shape

        cv2.putText(self.result, text='matching rate: {}'.format(int(matching_rate*1000)/1000), org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_4)
        
        # 検出した部分に赤枠をつける
        result_x = int(matching_loc[0]) + int(w/2)
        result_y = int(matching_loc[1]) + int(h/2)
        if result_x < int(self.w/2)-self.tolerance:
            cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 0, 255))
            cv2.circle(self.result, (result_x, result_y), 5, (0, 0, 255), 3)
            return 1 
        
        elif result_x > int(self.w/2)+self.tolerance:
            cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 0, 255))
            cv2.circle(self.result, (result_x, result_y), 5, (0, 0, 255), 3)
            return -1

        else:
            cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 255, 0))
            cv2.circle(self.result, (result_x, result_y), 5, (0, 255, 0), 3)
            return 0

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

    inst = module_belt_detect_for_c920(name)
    result = inst.run()
    print(result)
    inst.show_result()
