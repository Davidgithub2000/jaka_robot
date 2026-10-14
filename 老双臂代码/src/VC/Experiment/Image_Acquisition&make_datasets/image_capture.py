import os
import cv2
from Camera import Camera

root_path = os.getcwd()
cam = Camera()
image_data = root_path + '/' + 'images/'
os.makedirs(image_data, exist_ok=True)
image_id = len(os.listdir(image_data))

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
while True:
    image = cam.get_frame()
    cv2.imshow('image', image)
    k = cv2.waitKey(1)
    if k == 13:
        image_id += 1
        cv2.imwrite(image_data + 'im_data{}.png'.format(image_id), image)
    elif k == 27:
        cv2.destroyAllWindows()
        cam.release()
        break

