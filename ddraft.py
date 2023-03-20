import os
import csv

dir_path = os.path.realpath(os.path.dirname(__file__))

capture_timestamps = [1, 2, 3, 4, 5]

with open(f"{dir_path}/logs/capture_timestamps.csv", "w", newline="") as f:
            imuwriter = csv.writer(f)
            imuwriter.writerow(capture_timestamps)
