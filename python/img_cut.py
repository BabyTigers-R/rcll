import cv2

def onMouse(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, y)

img = cv2.imread("./images/ref_img.jpg", 0)
w = img.shape[1]
h = img.shape[0]

cv2.imshow("ref_img", img)
cv2.setMouseCallback("ref_img", onMouse)
cv2.waitKey(0)
cv2.destroyAllWindows()

x1 = int(input("x1: "))
y1 = int(input("y1: "))
x2 = int(input("x2: "))
y2 = int(input("y2: "))

img1 = img[y1:y2, x1:x2]

cv2.imshow("ref_img", img1)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite("./template_for_c920.png", img1)
