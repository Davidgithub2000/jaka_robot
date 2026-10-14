import cv2
import numpy as np
from Camrea import Camrea
import joblib


class ROI:
    def __init__(self):
        self.save_data = None
        self.pts = []
        self.cam = Camrea()
        self.image = None

    def get_roi_pts(self, event, x, y,flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pts.append((x, y))
        if event == cv2.EVENT_RBUTTONDOWN:
            self.pts.pop()
        if event == cv2.EVENT_MBUTTONDOWN:
            self.get_roi()

    def get_roi(self):
        mask = np.zeros_like(self.image, np.uint8)
        points = np.array(self.pts, np.int32)
        mask = cv2.fillPoly(mask, [points], (255, 255, 255))
        cv2.imshow('mask', mask)
        roi = cv2.bitwise_and(self.image, mask)
        cv2.imshow('ROI', roi)
        k = cv2.waitKey(0)
        if k == 27:
            self.save_data = {'ROI': self.pts}
            joblib.dump(value=self.save_data, filename='roi.pkl')
            cv2.destroyAllWindows()
            #self.cam.release()

    def main(self):
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.get_roi_pts)
        while True:
            image = self.cam.get_frame()
            self.image = image
            if len(self.pts) > 0:
                for i in self.pts:
                    cv2.circle(self.image, i, 2, (255, 255, 255), -1)
            if len(self.pts) > 1:
                for i in range(len(self.pts) - 1):
                    cv2.line(self.image, self.pts[i], self.pts[i + 1], (255, 255, 255), 2)
            cv2.imshow('image', self.image)
            k = cv2.waitKey(1)


if __name__ == '__main__':

    R = ROI()
    R.main()
