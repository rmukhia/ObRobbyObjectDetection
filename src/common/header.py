from enum import IntEnum, auto
from struct import pack, unpack

SIGNATURE = bytes('capture-video-server', 'utf-8')
HEADER_SIZE = 36
HEADER_FMT = '!20sILff'

class CaptureVideoCode(IntEnum):
    REPLY_ERROR = 1
    ''' Commands for frame capture '''
    REQUEST_CAPTURE = 20
    REPLY_CAPTURE = 21
    REPLY_CAPTURE_IMG = 22
    ''' Commands for movement '''
    REQUEST_MOVE_FORWARD = 70
    REQUEST_MOVE_LEFT = 71
    REQUEST_MOVE_RIGHT = 72
    REQUEST_MOVE_BACKWARD = 73
    REPLY_MOVE = 74

'''
|------------------------|---------------|----------------|-----------------|
| signature (20 bytes)   | cmd (4 bytes) | size (4 bytes) | speed (4 bytes) |
|------------------------|---------------|----------------|-----------------|
| time (4 bytes)         |
--------------------------
'''
def packHeader(cmd, size = 0, speed = 0, time = 0):
    header = pack(HEADER_FMT, SIGNATURE, cmd, size, speed, time)
    return header

def unpackHeader(data):
    header = unpack(HEADER_FMT, data)
    return header
