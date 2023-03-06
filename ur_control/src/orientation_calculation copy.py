import numpy as np
import urx
import time

# Define the reference orientation for the TCP
rob = urx.Robot("172.31.1.200")
lx = 0.1
ly = 0.1
lz = 0.3
v = 0.03
a = 0.1
r = 0.05
ref_orientation = rob.getl()

# Define the two 3D points
point_a = np.array([ref_orientation[0]+lx, ref_orientation[1]+ly, ref_orientation[2]+lz])
point_b = np.array([ref_orientation[0], ref_orientation[1], ref_orientation[2]])

pose_a = ref_orientation
pose_b = [ref_orientation[0]+lx, ref_orientation[1]+ly, ref_orientation[2]+lz, ref_orientation[3], ref_orientation[4], ref_orientation[5]]
print(type(pose_b))

# Calculate the direction vector between the two points
direction_vec = point_b - point_a

# Normalize the direction vector
unit_direction_vec = direction_vec / np.linalg.norm(direction_vec)

# Find the rotation matrix that rotates the reference orientation vector to the direction vector
cross_product_mat = np.array([[0, -unit_direction_vec[2], unit_direction_vec[1]],
                              [unit_direction_vec[2], 0, -unit_direction_vec[0]],
                              [-unit_direction_vec[1], unit_direction_vec[0], 0]])

dot_product = np.dot(ref_orientation[:3], unit_direction_vec)
cos_angle = dot_product / np.linalg.norm(ref_orientation[:3]) / np.linalg.norm(unit_direction_vec)
sin_angle = np.sqrt(1 - cos_angle**2)
rotation_mat = np.eye(3) * cos_angle + sin_angle * cross_product_mat + (1 - cos_angle) * np.outer(unit_direction_vec, unit_direction_vec)

# Apply the rotation matrix to the reference orientation vector to obtain the final orientation vector for the TCP
final_orientation_vec = np.concatenate((unit_direction_vec, np.zeros(3)))  # pad with zeros for roll, pitch, yaw
final_orientation_vec[3:] = np.matmul(rotation_mat, ref_orientation[3:])

# Print the final orientation vector in the format (x-translation, y-translation, z-translation, x-rotation, y-rotation, -z-rotation)
final_pose = tuple(np.round(np.concatenate((point_a, final_orientation_vec[3:]*180/np.pi)), 5))
final_pose = [final_pose[0], final_pose[1], final_pose[2], final_pose[3], final_pose[4], final_pose[5]]
print(f"Pose A: {pose_a}")
print(f"Pose B: {pose_b}")
print(f"Final Orientation: {final_pose}")

rob.movep(pose_b, acc=a, vel=v, wait=False)
while True:
    p = rob.getl(wait=True)
    if p[2] < final_pose[2] + 0.005:
        break

time.sleep(5)
print("Orienting...")

rob.movep(final_pose, acc=a, vel=v, wait=False)
while True:
    p = rob.getl(wait=True)
    if p[2] < final_pose[2] + 0.005:
        break
rob.close()
