import argparse
import socket
import asyncore
from multiprocessing.pool import ThreadPool
import cv2
from common import header , payload
from gpiozero import Robot
from time import sleep

# Capture class
cap = {}
threadPool = None
robot = None

def captureImage(index):
    ret, frame = cap[index].read()
    if ret:
        compressed_data = payload.compressData(frame)
        length = len(compressed_data)
        data = header.packHeader(header.CaptureVideoCode.REPLY_CAPTURE_IMG, length)
        packet = data + compressed_data
        return (packet, len(packet))
    return None

def moveRobot(cmd, speed, time):
    turning = False
    if cmd == header.CaptureVideoCode.REQUEST_MOVE_FORWARD:
        robot.forward(speed)
    elif cmd == header.CaptureVideoCode.REQUEST_MOVE_LEFT:
        robot.left(speed)
        turning = True
    elif cmd == header.CaptureVideoCode.REQUEST_MOVE_RIGHT:
        robot.right(speed)
        turning = True
    elif cmd == header.CaptureVideoCode.REQUEST_MOVE_BACKWARD:
        robot.backward(speed)
    if turning:
        sleep(time)
        robot.stop()
    return True


class ObRobbyHandler(asyncore.dispatcher_with_send):
    def buildPacket(self, len1, len2):
        packet = header.packHeader(header.CaptureVideoCode.REPLY_CAPTURE,  len1 + len2)
        return packet

    def replyCapture(self):
        data = threadPool.map(captureImage, (0, 1))
        for d in data:
            if not d:
                self.send(header.packHeader(header.CaptureVideoCode.REPLY_ERROR))
                return

        self.send(self.buildPacket(data[0][1], data[1][1]))
        self.send(data[0][0])
        self.send(data[1][0])

    def replyMove(self, cmd, speed, time):
        data = threadPool.starmap_async(moveRobot, ((cmd, speed, time),))
        #hdr = header.packHeader(header.CaptureVideoCode.REPLY_MOVE)
        #self.send(hdr)

    def handle_read(self):
        data = self.recv(header.HEADER_SIZE)
        if data:
            signature, cmd, size, speed, time = header.unpackHeader(data)
            if cmd == header.CaptureVideoCode.REQUEST_CAPTURE:
                self.replyCapture()
            if cmd >= header.CaptureVideoCode.REQUEST_MOVE_FORWARD and cmd <= header.CaptureVideoCode.REQUEST_MOVE_BACKWARD:
                self.replyMove(cmd, speed, time)
            

class ObRobbyServer(asyncore.dispatcher):
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket()
        self.set_reuse_addr()
        print('Server running on %s port %d' % (repr(host), port))
        self.bind((host, port))
        self.listen(5)

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print ('Incoming connection from %s' % repr(addr))
            handler = ObRobbyHandler(sock)

    def handle_close(self):
        robot.stop()
        self.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Video Server from robot')
    parser.add_argument('port', nargs=1, type=int,
            help='Port number')
    parser.add_argument('width', nargs=1, type=int,
            help='Width of image')
    parser.add_argument('height', nargs=1, type=int,
            help='Height of image')
    args = parser.parse_args()

    port = args.port[0]
    width = args.width[0]
    height = args.height[0]
    
    cap[0] = cv2.VideoCapture(0)
    cap[0].set(cv2.CAP_PROP_FRAME_WIDTH, width);
    cap[0].set(cv2.CAP_PROP_FRAME_HEIGHT, height);
    
    cap[1] = cv2.VideoCapture(1)
    cap[1].set(cv2.CAP_PROP_FRAME_WIDTH, width);
    cap[1].set(cv2.CAP_PROP_FRAME_HEIGHT, height);

    robot = Robot(left = (7, 8), right = (9, 10))

    threadPool = ThreadPool(processes = 8)
    server = ObRobbyServer('0.0.0.0', port)
    asyncore.loop()
