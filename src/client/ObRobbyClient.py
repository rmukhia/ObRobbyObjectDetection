import socket
from common import header , payload

class ObRobbyClient(object):
    def __init__(self, server, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((server, port))
        print('Connected to %s:%d' % self.sock.getpeername())

    def processImage(self, size):
        data = b'';
        dsize = 0;
        while(dsize < size):
            rsize = 4096
            if (size - dsize) < 4096:
                rsize = size - dsize
            data += self.sock.recv(rsize)
            dsize = len(data)

        header1offset = 0
        offset1 = header.HEADER_SIZE
        signature1, cmd1, size1, s , t = header.unpackHeader(data[header1offset: header1offset + header.HEADER_SIZE]) 
        header2offset = offset1 + size1
        img1 = payload.extractData(data[offset1: offset1 + size1])
        signature2, cmd2, size2, s ,t = header.unpackHeader(data[header2offset: header2offset + header.HEADER_SIZE])
        offset2 = header2offset + header.HEADER_SIZE
        img2 = payload.extractData(data[offset2: offset2 + size2])
        return (img1, img2)

    def sendReq(self, cmd, speed = 0, time = 0):
        self.sock.sendall(header.packHeader(cmd, 0, speed, time))

    def recvRep(self):
        data = self.sock.recv(header.HEADER_SIZE)
        signature, cmd, size, speed, time = header.unpackHeader(data)

        if (cmd == header.CaptureVideoCode.REPLY_CAPTURE):
            return self.processImage(size)

    def close(self):
        self.sock.close()
