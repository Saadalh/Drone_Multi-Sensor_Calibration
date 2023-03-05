import urx
import time
import numpy as np

if __name__ == "__main__":
    rob = urx.Robot("172.31.1.200")
    try:
        l = 0.1
        v = 0.03
        a = 0.1
        r = 0.05
        pose = rob.getl()
        """"
        pose[2] += l
        rob.movep(pose, acc=a, vel=v, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[2] > pose[2] - 0.05:
                break
        print("Move 1 is done.")

        pose[1] += l 
        rob.movep(pose, acc=a, vel=v, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[1] > pose[1] - 0.05:
                break
        print("Move 2 is done.")

"""
        pose[2] += l
        rob.movep(pose, acc=a, vel=v, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[2] < pose[2] + 0.005:
                break
        #pose[1] -= l
        #rob.movep(pose, acc=a, vel=v, wait=True)
        #print("Move 4 is done.")

    finally:
        rob.close()
