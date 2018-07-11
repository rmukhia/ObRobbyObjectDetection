import cv2
import numpy as np
import argparse
from client.ObRobbyClient import ObRobbyClient
from client import ObRobbyImageProcess

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Cature image for calibration')
    parser.add_argument('host', nargs=1, type=str,
            help='Host number')
    parser.add_argument('port', nargs=1, type=int,
            help='Port number')
    parser.add_argument('camera', nargs=1, type=int,
            help='should be 0/1')
    parser.add_argument('count', nargs=1, type=int,
            help='number of images')
    args = parser.parse_args()

    client = ObRobbyClient(args.host[0],args.port[0])
    
    cv2.namedWindow('feed')

    for i in range(0, args.count[0]* 20):
        imgL, imgR = ObRobbyImageProcess.captureImage(client, args.camera[0])
        image = np.concatenate((imgL, imgR), axis = 1)
        cv2.imshow('feed', image)
        cv2.waitKey(1)
        if i % 20 == 0:
            filenameL = 'output/LEFT/image%d.png' % (i/20)
            filenameR = 'output/RIGHT/image%d.png' % (i/20)
            cv2.imwrite(filenameL, imgL)
            cv2.imwrite(filenameR, imgR)
            print('%s %s' % (filenameL, filenameR))
    client.close()
