import cv2
import numpy as np
import argparse
from client.ObRobbyClient import ObRobbyClient
from client.ObRobbyDetectObject import ObRobbyDetectObject
from client.ObRobbyMovement import ObRobbyMovement
from client import ObRobbyIO, ObRobbyImageProcess


def detectObstacle(imgL, imgR, camera_obj):
    detector = ObRobbyDetectObject(imgL, imgR, camera_obj)
    found = detector.findFeatures()
    result = False
    if found:
        detector.projectTo3D()
        result = detector.detectObstacle(distanceThreshold = 200, depthThreshold = 15, areaThreshold = 80)
    detector.drawKeypoints('Features')
    detector.drawDepth(20, 20, 'Depth')
    detector.drawObstacle('Obstacle')
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Engine on the cloud')
    parser.add_argument('host', nargs=1, type=str,
            help='Host number')
    parser.add_argument('port', nargs=1, type=int,
            help='Port number')
    parser.add_argument('camera', nargs=1, type=int,
            help='should be 0/1')
    parser.add_argument('-debug',  action='store_true')
    args = parser.parse_args()

    camera_obj = ObRobbyIO.getCameraCalibrationProp('../data/calibration_data')

    cv2.namedWindow('VideoFeed')
    cv2.namedWindow('Features')
    cv2.namedWindow('Depth')
    cv2.namedWindow('Obstacle')

    client = ObRobbyClient(args.host[0], args.port[0])

    movementClient = ObRobbyClient(args.host[0], args.port[0])

    movement = ObRobbyMovement(movementClient, debug = args.debug)

    while True:
        imgL, imgR = ObRobbyImageProcess.captureImage(client, args.camera[0])
        imgL, imgR = ObRobbyImageProcess.toGrayScale(imgL, imgR)
        imgL, imgR = ObRobbyImageProcess.normalizeImage(imgL, imgR)

        videoFeed = np.concatenate((imgL, imgR), axis = 1)
        cv2.imshow('VideoFeed', videoFeed)

        imgL, imgR = ObRobbyImageProcess.rectifyImage(imgL, imgR, camera_obj)

        obstacleFound = detectObstacle(imgL, imgR, camera_obj)
        cv2.waitKey(1)
        if obstacleFound:
            movement.move(movement.AVOID)
        else:
            movement.move(movement.PROCEED)

    client.close()
