import pickle

def getCameraCalibrationProp(filename):
    with open(filename, 'rb') as infile:
        print(infile)
        cal_calib = pickle.load(infile)
    return cal_calib
