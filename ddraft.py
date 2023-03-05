import numpy as np
chessboardSize = (9,6)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 20

print(objp)
objp = objp * size_of_chessboard_squares_mm

#print(objp)