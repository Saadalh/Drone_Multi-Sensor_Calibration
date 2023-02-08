import urx
import numpy as np

if __name__ == "__main__":
    rob = urx.Robot("192.168.56.1")
    #rob = urx.Robot("localhost")
    rob.set_payload(0.5, (0, 0, 0))

    # get and print the cuurent pose of the tcp 
    pose = rob.getl()
    print(pose) # should be in x, y, z, w, x, y, z format

    # convert to a transformation matrix
    x, y, z = pose[0:3]
    w, i, j, k = pose[3:7]

    R = np.array([[1 - 2 * (j**2 + k**2), 2 * (i * j - k * w), 2 * (i * k + j * w)],
              [2 * (i * j + k * w), 1 - 2 * (i**2 + k**2), 2 * (j * k - i * w)],
              [2 * (i * k - j * w), 2 * (j * k + i * w), 1 - 2 * (i**2 + j**2)]])

    T = np.array([[R[0, 0], R[0, 1], R[0, 2], x],
                  [R[1, 0], R[1, 1], R[1, 2], y],
                  [R[2, 0], R[2, 1], R[2, 2], z],
                  [0      , 0      , 0      , 1]])
