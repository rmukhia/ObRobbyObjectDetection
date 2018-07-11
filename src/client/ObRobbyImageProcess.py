import cv2
import numpy as np
import math
from timeit import default_timer as timer
from common import header

def captureImage(client, camera_placement = 1):
    # start = timer()
    client.sendReq(header.CaptureVideoCode.REQUEST_CAPTURE)
    imgL = None
    imgR = None
    if camera_placement == 0:
        imgL, imgR = client.recvRep()
    else:
        imgR, imgL = client.recvRep()
    # end = timer()
    # print('Time to capture %f sec.' % (end - start))
    return imgL, imgR


def rectifyImage(imgL, imgR, camera_dict, windowname=None):
    left_map1, left_map2 = camera_dict['camera_prop']['left_map']
    right_map1, right_map2 = camera_dict['camera_prop']['right_map']
    rimgL = cv2.remap(imgL, left_map1, left_map2, 4) 
    rimgR = cv2.remap(imgR, right_map1, right_map2, 4) 
    if windowname:
        image = np.concatenate((rimgL, rimgR), axis = 1)
        cv2.imshow(windowname, image)
    return rimgL, rimgR;

'''
    The number of images should be odd
'''
def denoiseImage(imgL, imgR, hCl =3):
    imgLl = cv2.fastNlMeansDenoising(imgL, h = hCl)
    imgLr = cv2.fastNlMeansDenoising(imgR, h = hCl)
    return imgLl, imgLr

def toGrayScale(imgL, imgR):
    return cv2.cvtColor(imgL, cv2.COLOR_RGB2GRAY), cv2.cvtColor(imgR, cv2.COLOR_RGB2GRAY)

def normalizeImage(imgL, imgR):
    return cv2.normalize(imgL, None, 1, 255, cv2.NORM_MINMAX), cv2.normalize(imgR, None, 1, 255, cv2.NORM_MINMAX)

        

