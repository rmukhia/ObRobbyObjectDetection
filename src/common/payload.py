import pickle
import zlib

MOVEMENT_PAYLOAD_SIZE = 8
MOVEMENT_PAYLOAD_FMT = '!ff'

def createMovementPayload(speed, time):
    header = pack(MOVEMENT_PAYLOAD_SIZE, speed, time)
    return header

def extractMovementPayload(data):
    header = unpack(MOVEMENT_PAYLOAD_SIZE, data)
    return header

def compressData(data):
    serialized = pickle.dumps(data)
    # compressed_data = zlib.compress(serialized)
    return serialized

def extractData(data):
    # extracted_data = zlib.decompress(data)
    deserialized = pickle.loads(data)
    return deserialized
