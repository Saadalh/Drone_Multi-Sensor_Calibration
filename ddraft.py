import cv2
import numpy as np
import os
import csv
from scipy.spatial.transform import Rotation as R

rvecs = R.from_rotvec(np.array([-0.63332594, 0.30748745, -3.04953911]))
rvec = np.array([-0.63332594, 0.30748745, -3.04953911])

rvector = cv2.Rodrigues(rvec)
rvectors = rvecs.as_matrix()

tvec = [1, 2, 3]
rvec = [4,5,6]
robpose = []

robpose.append(tvec,rvec)

dir_path = os.path.realpath(os.path.dirname(__file__))

with open(f"{dir_path}/draft.csv", "w", newline="") as f:
        imuwriter = csv.writer(f)
        imuwriter.writerow(robpose)