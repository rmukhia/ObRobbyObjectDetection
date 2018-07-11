import numpy as np
import cv2
import glob
import argparse
import pickle


class StereoCalibration(object):

    def __init__(self, filepath):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((7 * 5, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_l = []  # 2d points in image plane.
        self.imgpoints_r = []  # 2d points in image plane.

        self.cal_path = filepath
        self.blurredThreshold = 100.0
        self.read_images(self.cal_path)

    def isBlurred(self, frame):
        # return (cv2.Laplacian(frame, cv2.CV_64F).var() >=
        # self.blurredThreshold)
        return False

    def read_images(self, cal_path):
        images_right = glob.glob(cal_path + 'RIGHT/*.png')
        images_left = glob.glob(cal_path + 'LEFT/*.png')
        images_left.sort()
        images_right.sort()

        for i, fname in enumerate(images_right):
            img_l = cv2.imread(images_left[i])
            img_r = cv2.imread(images_right[i])

            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (7, 5), None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (7, 5), None)


            if ret_l is True and ret_r is True and not self.isBlurred(gray_l) and not self.isBlurred(gray_r):
                rt = cv2.cornerSubPix(gray_l, corners_l, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_l.append(corners_l)

                # Draw and display the corners
                ret_l = cv2.drawChessboardCorners(img_l, (7, 5),
                                                  corners_l, ret_l)
                cv2.imshow(images_left[i], img_l)
                cv2.waitKey(500)
                rt = cv2.cornerSubPix(gray_r, corners_r, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_r.append(corners_r)

                # Draw and display the corners
                ret_r = cv2.drawChessboardCorners(img_r, (7, 5),
                                                  corners_r, ret_r)
                cv2.imshow(images_right[i], img_r)
                cv2.waitKey(500)
                # If found, add object points, image points (after refining them)
                self.objpoints.append(self.objp)
            img_shape = gray_l.shape[::-1]

        rt, self.M1, self.d1, self.r1, self.t1 = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_l, img_shape, None, None)
        rt, self.M2, self.d2, self.r2, self.t2 = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_r, img_shape, None, None)

        self.camera_model = self.stereo_calibrate(img_shape)
        self.camera_prop = self.calculateStereo(self.camera_model, img_shape)
        self.camera_variables = dict(
            [('camera_model', self.camera_model), ('camera_prop', self.camera_prop)])

    def stereo_calibrate(self, dims):
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_l,
            self.imgpoints_r, self.M1, self.d1, self.M2,
            self.d2, dims,
            criteria=stereocalib_criteria, flags=flags)

        print('Intrinsic_mtx_1', M1)
        print('dist_1', d1)
        print('Intrinsic_mtx_2', M2)
        print('dist_2', d2)
        print('R', R)
        print('T', T)
        print('E', E)
        print('F', F)

        # for i in range(len(self.r1)):
        #     print("--- pose[", i+1, "] ---")
        #     self.ext1, _ = cv2.Rodrigues(self.r1[i])
        #     self.ext2, _ = cv2.Rodrigues(self.r2[i])
        #     print('Ext1', self.ext1)
        #     print('Ext2', self.ext2)

        print('')

        camera_model = dict([('M1', M1), ('M2', M2), ('dist1', d1),
                             ('dist2', d2), ('rvecs1', self.r1),
                             ('rvecs2', self.r2), ('R', R), ('T', T),
                             ('E', E), ('F', F)])

        cv2.destroyAllWindows()
        return camera_model

    def calculateStereo(self, model, img_shape):
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(model['M1'], model['dist1'], 
                                                        model['M2'], model['dist2'], img_shape,
                                                        model['R'], model['T'])
        left_map = cv2.initUndistortRectifyMap(
            model['M1'], model['dist1'], R1, P1, img_shape, cv2.CV_16SC2)
        right_map = cv2.initUndistortRectifyMap(
            model['M2'], model['dist2'], R2, P2, img_shape, cv2.CV_16SC2)
        camera_prop = dict([('R1', R1), ('R2', R2), ('P1', P1),
                            ('P2', P2), ('Q', Q),
                            ('roi1', roi1), ('roi2', roi2),
                            ('left_map', left_map), ('right_map', right_map)])
        return camera_prop


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filepath', help='String Filepath')
    args = parser.parse_args()
    cal_data = StereoCalibration(args.filepath)
    with open('calibration_data', 'wb') as outfile:
        pickle.dump(cal_data.camera_variables, outfile)
