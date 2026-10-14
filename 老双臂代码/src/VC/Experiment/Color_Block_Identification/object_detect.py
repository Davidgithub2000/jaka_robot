import cv2
import numpy as np
from Camrea import Camrea
import joblib


class Detect:
    def __init__(self, color_select: str = 'red'):
        self.color = {'red': np.array([[[0, 43, 46], [10, 255, 255]],
                                      [[156, 43, 46], [180, 255, 255]]]),
                      'blue': np.array([[85, 30, 46], [124, 255, 255]]),
                      'yellow': np.array([[11, 43, 46], [34, 255, 255]]),
                      }
        self.area = {'lower': 6000, 'upper': 9000}
        self.scale_w_h = {'lower': 0.8, 'upper': 1.2}
        self.color_select = color_select

    @staticmethod
    def get_roi(image): # 获取ROI图像
        mask = np.zeros_like(image, np.uint8)
        pts = joblib.load('roi.pkl')['ROI']
        points = np.array(pts, np.int32)
        mask = cv2.fillPoly(mask, [points], (255, 255, 255))
        image_roi = cv2.bitwise_and(image, mask)
        return image_roi

    def detect(self, image_roi):
        img_hsv = cv2.cvtColor(image_roi, cv2.COLOR_BGR2HSV)
        if self.color_select == 'red':
            th_img0 = cv2.inRange(img_hsv, lowerb=self.color[self.color_select][0][0], upperb=self.color[self.color_select][0][1])
            th_img1 = cv2.inRange(img_hsv, lowerb=self.color[self.color_select][1][0], upperb=self.color[self.color_select][1][1])
            th = cv2.bitwise_or(th_img1, th_img0)
        else:
            th = cv2.inRange(img_hsv, lowerb=self.color[self.color_select][0], upperb=self.color[self.color_select][1])
        mask = np.zeros_like(th, dtype=np.uint8)
        cnts, he = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        boxes, centers = [], []
        for i, cnt in enumerate(cnts):
            area = cv2.contourArea(cnt)
            (x, y, w, h) = cv2.boundingRect(cnt)
            bound_area = w * h
            scale_w_h = w / h
            #print(bound_area)
            if self.area['lower'] < bound_area < self.area['upper'] and self.scale_w_h['lower'] < scale_w_h < self.scale_w_h['upper']:
                cv2.drawContours(mask, cnt, -1, 255, 1)
                box = [x, y, x+w, y+h]
                center_x, center_y = x+w//2, y+h//2
                boxes.append(box)
                centers.append([center_x, center_y])
                cv2.drawContours(image_roi, cnt, -1, (255,255,255), 2)
        return boxes, centers,th, mask

if __name__ == '__main__':
    cam = Camrea()
    D = Detect(color_select='red')
    cv2.namedWindow('image_all', cv2.WINDOW_NORMAL)
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    while True:
        image = cam.get_frame()
        image_roi = D.get_roi(image)
        boxes, centers, th, mask = D.detect(image_roi)
        image_all = np.hstack((th, mask))
        cv2.imshow('image_all', image_all)
        cv2.imshow('image', image_roi)
        k = cv2.waitKey(1)
        if k == 27:
            cv2.destroyAllWindows()
            cam.release()

