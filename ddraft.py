import csv

list1 = [12, 13]
list2 = [21, 22]
list3 = [32, 33]

listoflists = []

listoflists.append(["IMU Start", " IMU End"])
listoflists.append(list1)
listoflists.append(list2)
listoflists.append(list3)

with open("draftlistoflists.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(listoflists)