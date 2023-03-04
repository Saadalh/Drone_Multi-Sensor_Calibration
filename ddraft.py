import os 

dir_path = os.path.realpath(os.path.dirname(__file__))

# Create logging file
filename = dir_path + "/../urx/test.txt"
f = open(filename, "w")
line = "hallo. hier ist ein test text."
f.write(line)
f.close()