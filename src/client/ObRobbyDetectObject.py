import math
import cv2
import numpy as np

class ObRobbyDetectObject(object):
    def __init__(self, imgL, imgR, camera_obj):
        self.camera_obj = camera_obj
        self.imgL = imgL
        self.imgR = imgR
        self.surf = cv2.xfeatures2d.SURF_create(20)
        self.surf.setUpright(True)
        self.kpL = None
        self.kpR = None
        self.desL = None
        self.desR = None
        self.ptsL = []
        self.ptsR = []
        self.distance = []
        self.F = None
        self.depth = []
        self.obstacleFound = False

    def _calculateDistance(self):
        # x1 - x2 , y1 - y2
        a = self.ptsL - (self.ptsR + [self.imgL.shape[1], 0])
        # (x1 - x2)^2, (y1 - y2)^2
        b = np.multiply(a, a)
        # (x1 - x2)^2 + (y1 - y2)^2
        return np.sqrt(np.sum(b, axis = 1))

    def findFeatures(self):
        self.kpL, self.desL = self.surf.detectAndCompute(self.imgL, None)
        self.kpR, self.desR = self.surf.detectAndCompute(self.imgR, None)

        if type(self.desL).__name__ == 'NoneType' or type(self.desR).__name__ == 'NoneType':
            return False
        # FLANN parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 12)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params,search_params)

        if(len(self.kpL)>=2 and len(self.kpR)>=2): 
            matches = flann.knnMatch(self.desL, self.desR ,k=2)
        else:
            return False

        good = []
        # ratio test as per Lowe's paper
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.8*n.distance:
                good.append(m)
                self.ptsR.append(self.kpR[m.trainIdx].pt)
                self.ptsL.append(self.kpL[m.queryIdx].pt)


        otmPtsL = []
        otmPtsR = []
        for ptL, ptR in zip(self.ptsL, self.ptsR):
            if (abs(ptL[1] - ptR[1]) < 5.0):
                otmPtsL.append(ptL)
                otmPtsR.append(ptR)


        self.ptsL = np.int32(otmPtsL)
        self.ptsR = np.int32(otmPtsR)
        # Need 9 points to find fundamental matrix
        if self.ptsR.shape[0] <= 4 or self.ptsL.shape[0] <= 4:
            return False

        self.F, mask = cv2.findFundamentalMat(self.ptsL, self.ptsR, cv2.FM_LMEDS)

        if self.F is None or self.F.shape == (1, 1):
            return False
        elif self.F.shape[0] > 3:
            self.F = self.F[0:3, 0:3]

        self.ptsL = self.ptsL[mask.ravel()==1]
        self.ptsR = self.ptsR[mask.ravel()==1]

        self.distance = self._calculateDistance()
        return True


    def projectTo3D(self):
        P1 = self.camera_obj['camera_prop']['P1']
        P2 = self.camera_obj['camera_prop']['P2']

        ptsL = np.zeros((2, self.ptsL.shape[0]))
        ptsR = np.zeros((2, self.ptsR.shape[0]))
        for i,(ptL, ptR) in enumerate(zip(self.ptsL, self.ptsR)):
            ptsL[0, i] = ptL[0]
            ptsL[1, i] = ptL[1]
            ptsR[0, i] = ptR[0]
            ptsR[1, i] = ptR[1]

        self.pts3d = cv2.triangulatePoints(P1, P2, ptsL, ptsR)
        pts3d = []
        for i in range(0, self.pts3d.shape[1]):
            pts3d.append([self.pts3d[0, i], self.pts3d[1, i], self.pts3d[2, i], self.pts3d[3,i]])
        self.pts3d = np.array(pts3d)
        self.pts3d = cv2.convertPointsFromHomogeneous(self.pts3d)
        self.pts3d = np.reshape(self.pts3d, (self.pts3d.shape[0], 3))
        self.depth = self.pts3d[:,2]

        # Filter out negative depth
        posDepth = self.depth > 0
        self.ptsL = self.ptsL[posDepth]
        self.ptsR = self.ptsR[posDepth]
        self.pts3d = self.pts3d[posDepth]
        self.depth = self.depth[posDepth]
        self.distance = self.distance[posDepth]


    def detectObstacle(self, distanceThreshold = 100, depthThreshold = 15, areaThreshold = 25):
        distIndex = self.distance <= distanceThreshold
        distance = self.distance[distIndex]
        depth = np.mean(self.depth[distIndex])
        ptsL = self.ptsL[distIndex]
        ptsR = self.ptsR[distIndex]
        
        if ptsL.shape[0] > 0 and ptsR.shape[0] > 0 and depth <= depthThreshold:
            self.boundingRectL = cv2.boundingRect(ptsL)
            self.boundingRectR = cv2.boundingRect(ptsR)
            self.meanDepth = depth
            _ ,_ , w, h = self.boundingRectL
            if (w * h) >= areaThreshold:
                self.obstacleFound = True
                return True

        self.obstacleFound = False
        return False


    def drawKeypoints(self, windowname = None):
        kpL = [cv2.KeyPoint(pt[0], pt[1], 1) for pt in self.ptsL]
        kpR = [cv2.KeyPoint(pt[0], pt[1], 1) for pt in self.ptsR]
        matches = [cv2.DMatch(i, i, 0) for i in range(0, len(self.ptsL))]

        image = cv2.drawMatches(self.imgL, kpL, self.imgR, kpR, matches, None)
        for dist, pt in zip(self.distance, self.ptsL):
            image = cv2.putText(image, '%0.2f' % (dist), (pt[0], pt[1]), cv2.FONT_HERSHEY_PLAIN, 0.7, (50,50,255))
        if windowname:
            cv2.imshow(windowname, image)
        return image

    def _getRadius(self, depth, maxRadius):
        d = maxRadius - depth
        if d > maxRadius:
            d = maxRadius
        elif d < 1:
            d = 1
        return math.floor(d/3 * d/3)

    def drawDepth(self, maxRadius = 20, numCircles=20, windowname = None):
        rimgL = np.copy(self.imgL)
        rimgR = np.copy(self.imgR)
        index = np.argsort(self.depth)

        if numCircles > index.shape[0]:
            numCircles = index.shape[0]

        for i in range(0, numCircles):
            ptL = self.ptsL[index[i]]
            ptR = self.ptsR[index[i]]
            depth = self.depth[index[i]]
            pL = (ptL[0], ptL[1])
            pR = (ptR[0], ptR[1])
            rimgL = cv2.circle(rimgL, pL, self._getRadius(depth, maxRadius), (166, 166, 166))
            rimgL = cv2.putText(rimgL, '%0.2f' % (depth), pL, cv2.FONT_HERSHEY_PLAIN, 0.7, (166,166,166))
            rimgR = cv2.circle(rimgR, pR, self._getRadius(depth, maxRadius), (166, 166, 166))
            rimgR = cv2.putText(rimgR, '%0.2f' % (depth), pR, cv2.FONT_HERSHEY_PLAIN, 0.7, (166,166,166))

        if windowname:
            image = np.concatenate((rimgL, rimgR), axis = 1)
            cv2.imshow(windowname, image)

        return rimgL, rimgR

    def drawObstacle(self, windowname = None):
        imgL = cv2.cvtColor(np.copy(self.imgL), cv2.COLOR_GRAY2RGB)
        imgR = cv2.cvtColor(np.copy(self.imgR), cv2.COLOR_GRAY2RGB)
        if self.obstacleFound:
            x,y,w,h = self.boundingRectL
            imgL =cv2.rectangle(imgL, (x, y), (x+w, y+h), (0, 0, 255), 2)
            imgL = cv2.putText(imgL, 'Dist: %0.2f' % (self.meanDepth), (math.floor(x + w/2), math.floor(y + h/2)), cv2.FONT_HERSHEY_PLAIN, 0.7, (0,0,255))
            x,y,w,h = self.boundingRectR
            imgR = cv2.rectangle(imgR, (x, y), (x+w, y+h), (0, 0, 255), 2)
            imgR = cv2.putText(imgR, 'Dist: %0.2f' % (self.meanDepth), (math.floor(x + w/2), math.floor(y + h/2)), cv2.FONT_HERSHEY_PLAIN, 0.7, (0,0,255))

        if windowname:
            image = np.concatenate((imgL, imgR), axis = 1)
            if self.obstacleFound:
                image = cv2.putText(image, 'AVOID', (30, 40), cv2.FONT_HERSHEY_PLAIN , 1.5, (0, 0 ,255))
            else:
                image = cv2.putText(image, 'PROCEED', (30, 40), cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255 ,0))
            cv2.imshow(windowname, image)

        return imgL, imgR

