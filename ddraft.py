import glob 
import os


dir_path = os.path.realpath(os.path.dirname(__file__))

capture_timestamps = [1679331196.7858083,
1679331210.4203894,
1679331223.1596243,
1679331235.481563,
1679331248.0723314,
1679331258.9359348,
1679331270.6875293,
1679331282.5411067,
1679331293.986442,
1679331305.882917,
1679331317.640924,
1679331328.8929067,
1679331340.1009638,
1679331352.653874,
1679331364.86798,
1679331377.9480028,
1679331391.8654845,
1679331402.3897486,
1679331411.8048804,
1679331424.1186152,
1679331437.5287662,
1679331450.2485774,
1679331463.8483784,
1679331475.214464,
1679331487.435546,
1679331499.101554,
1679331511.4437685] # 27

all_captures = glob.glob(f'{dir_path}/logs/captures/*.jpg')
capture_files = []
for ts in capture_timestamps:
    timediff = 10
    wantedcapnum = 0
    wantedcapts = 0
    for scap in all_captures:
        caphead, captail = os.path.split(scap)
        uscount = 0
        dcount = 0
        capts = ""
        capnum = ""
        for c in captail:
            if c == ".":
                dcount += 1
            if uscount == 2 and dcount < 2:
                capts += c 
            if uscount == 1 and dcount == 0:
                capnum += c
            if c == "_":
                uscount += 1
        if abs(float(capts)-ts) < timediff:
            timediff = abs(float(capts)-ts)
            wantedcapts = capts
            wantedcapnum = capnum
    capture_files.append(f"{dir_path}/logs/captures/img_{wantedcapnum}{wantedcapts}.jpg")

for cfile in all_captures:
    if cfile not in capture_files:
        os.remove(cfile)