import numpy as np
import cv2 as cv
import glob
import os

path_images = '/home/cs3630/Documents/camera-cal-gopro/frames'

images = glob.glob(os.path.join(path_images, '*.png'))

row = 7
col = 9

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((row*col,3), np.float32)
objp[:,:2] = np.mgrid[0:col,0:row].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (col,row), None)

    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        cv.drawChessboardCorners(img, (col,row), corners2, ret)

        # img_sm = cv.resize(img, (int(4000/2), int(2130/2)))
        # cv.imshow('img', img_sm)
        # cv.waitKey(0)


ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(ret, mtx, dist) #, rvecs, tvecs)


##
img = cv.imread(os.path.join(path_images, '5.png'))
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))


# ##
# dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# # crop the image
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]
# cv.imwrite('calibresult.png', dst)