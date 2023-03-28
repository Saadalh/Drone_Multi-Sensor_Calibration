import glob
import os
import re

def get_capture_number(filename):
    # Extracts the capture number from the file name
    # Assumes the file name format is '/path/to/file/img_{capture_number}_timestamp.jpg'
    match = re.search(r'img_(\d+)_\d+\.\d+\.jpg', filename)
    if match:
        return int(match.group(1))
    else:
        return -1

dir_path = os.path.realpath(os.path.dirname(__file__))
captures = glob.glob(f"{dir_path}/logs/4x6_captures/*.jpg")

sorted_filenames = sorted(captures, key=get_capture_number)

for i in sorted_filenames:
    print(i)
