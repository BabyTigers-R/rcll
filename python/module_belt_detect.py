import numpy as np
import cv2
import sys
import time
# from module_photographer import module_photographer

class module_belt_detect():
    def __init__(self):
        self.templ = cv2.imread("./templ/template02.png", 0)
        #gray = cv2.imread("{}_gray.jpg".format(self.name), 0)
        self.tolerance = 5

    def __call__(self, img):
        # 処理対象画像に対して、テンプレート画像との類似度を算出する
        self.img = img
        self.result = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        self.w = self.img.shape[1]
        self.h = self.img.shape[0]
        self.res = cv2.matchTemplate(self.img, self.templ, cv2.TM_CCOEFF_NORMED)
        self.ref_line = int(self.w/2) # - 120

        # 類似度の高い部分を検出する
        # threshold = 0.60
        threshold = 0.56
        # threshold = 0.50
        #print(res.dtype)
        loc = np.where(self.res == np.max(self.res))
        matching_loc = np.array([loc[1][0], loc[0][0]])
        matching_rate = np.max(self.res)
        print(matching_rate)

        if matching_rate < threshold:
            cv2.putText(self.result, text='No matching location', org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_4)
            self.show_result()
            return 2

        # テンプレートマッチング画像の高さ、幅を取得する
        h, w = self.templ.shape

        cv2.putText(self.result, text='matching rate: {}'.format(int(matching_rate*1000)/1000), org=(0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_4)
        
        # 検出した部分に赤枠をつける
        result_x = int(matching_loc[0]) + int(w/2)
        result_y = int(matching_loc[1]) + int(h/2)

        # left
        if result_x < int(self.ref_line)-self.tolerance:
            cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 0, 255))
            cv2.circle(self.result, (result_x, result_y), 5, (0, 0, 255), 3)
            
            self.show_result()
            return 1
        
        # right 
        elif result_x > int(self.ref_line)+self.tolerance:
            cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 0, 255))
            cv2.circle(self.result, (result_x, result_y), 5, (0, 0, 255), 3)
            
            self.show_result()
            return -1

        # center
        else:
            cv2.line(self.result, (int(self.ref_line), 0), (int(self.ref_line), self.h), (0, 255, 0))
            cv2.circle(self.result, (result_x, result_y), 5, (0, 255, 0), 3)

            self.show_result()
            return 0

    def show_result(self):
        # 画像の保存
        cv2.imwrite('./images/belt.png', self.result)

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

    bd = module_belt_detect()
    error = bd(gray)
    print(error)
    #result = inst()
    #print(result)
    #inst.show_result()
