import urx
import time
import numpy as np

if __name__ == "__main__":
    rob = urx.Robot("172.31.1.200")
    #rob = urx.Robot("localhost")
    rob.set_payload(0.5, (0, 0, 0))
    time.sleep(0.2)

    # get and print the cuurent pose of the tcp 
    pose = rob.getl()
    print(pose) # should be in x, y, z, w, x, y, z format

    # convert to a transformation matrix
    x, y, z = pose[0:3]
    roll, pitch, yaw = pose[3:6]

    pose[5] += 1

    print(pose)

    rob.movep(pose, acc=0.1, vel=0.07, wait=True)

    time.sleep(10)

    rob.close()

    #print(f"----------------------\nRotation Matrix:\n{R}\n\nTransformation Matrix:\n{T}")
