import cv2

def calib_hand_eye(r_1to2, t_1to2, r_3to4, t_3to4, hec_method):
    r_4to1, t_4to1 = cv2.calibrateHandEye(r_1to2, t_1to2, r_3to4, t_3to4, method=hec_method)
    print(t_4to1)
    print("#################")
    print(r_4to1)