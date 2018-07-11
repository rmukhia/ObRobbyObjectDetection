import cv2
from client.ObRobbyClient import ObRobbyClient
from client import ObRobbyIO, ObRobbyImageProcess
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Display Image from ObRobby')
    parser.add_argument('host', nargs=1, type=str,
            help='Host number')
    parser.add_argument('port', nargs=1, type=int,
            help='Port number')
    parser.add_argument('camera', nargs=1, type=int,
            help='should be 0/1')
    args = parser.parse_args()

    camera_obj = ObRobbyIO.getCameraCalibrationProp('../data/calibration_data')

    cv2.namedWindow('feed')

    client = ObRobbyClient(args.host[0], args.port[0])

    while True:
        imgL, imgR = ObRobbyImageProcess.captureImage(client, args.camera[0])
        rimgL, rimgR = ObRobbyImageProcess.rectifyImage(imgL, imgR, camera_obj, 'feed')
        cv2.waitKey(1)
    client.close()
