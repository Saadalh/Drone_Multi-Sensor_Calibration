import cv2
import time
import argparse
import threading
import cflib.crtp
import numpy as np
import socket, os, struct
from cflib.crazyflie import Crazyflie

# Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument('--unsave', action='store_false', help="Dont save streamed images")
args = parser.parse_args()

# Radio, WiFi connection, and Path variables
deck_port = args.p
deck_ip = args.n

class Camera:

    save = False
    stream = True
    # WiFi connection method
    # If it didn't work outdent lines 31,32,33, and change main accordingly.
    def __init__(self, deck_ip, deck_port):
        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print("Socket connected")

    # Image raw-pack information setup
    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data))) # similar to append()
        return data

    # Continuous frames acquisition method
    def stream_video(self, start, count, dir_path):
        time.sleep(0.1)
        while(self.stream):
            print(self.save)
            packetInfoRaw = self.rx_bytes(4)
            #print(packetInfoRaw)
            [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
            #print("Length is {}".format(length))
            #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
            #print("Function is 0x{:02X}".format(function))

            imgHeader = self.rx_bytes((length - 2))
            #print(imgHeader)
            #print("Length of data is {}".format(len(imgHeader)))
            [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

            if magic == 0xBC:
            #print("Magic is good")
            #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            #print("Image format is {}".format(format))
            #print("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
                imgStream = bytearray() # Getting stuck here
                print("STIUCK")
                
                while len(imgStream) < size:
                    packetInfoRaw = self.rx_bytes(4)
                    [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                    #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                    chunk = self.rx_bytes((length - 2))
                    imgStream.extend(chunk)
                
                count = count + 1
                meanTimePerImage = (time.time()-start) / count
                #print("{}".format(meanTimePerImage))
                #print("{}".format(1/meanTimePerImage))

                filename = dir_path+f"/../captures/img_{count:06d}_"+str(start)+".jpg"
                if format == 0:
                    bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                    bayer_img.shape = (244, 324)
                    #cv2.imshow('Raw', bayer_img)
                    if self.save:
                        if not cv2.imwrite(filename, bayer_img):
                            raise Exception("Could not save capture image")
                    self.save = False
                    cv2.waitKey(1)
            else:
                with open("img.jpeg", "wb") as f:
                    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                cv2.imshow('JPEG', decoded)
                cv2.waitKey(1)

    # On-demand single frame capturing method
    def capture(self):
        time.sleep(5)
        print("CAPTURE!")
        self.save = True

        time.sleep(5)
        print("CAPTURE!")
        self.save = True

        self.stream = False

if __name__ == '__main__':
    direct_path = os.path.realpath(os.path.dirname(__file__))
    # Initialize the low-level cflib drivers
    cflib.crtp.init_drivers()
    camObj = Camera(deck_ip, deck_port)
    count = 0

    video_thread = threading.Thread(target=camObj.stream_video, args=(time.time(), count, direct_path))
    video_thread.start()

    capture_thread = threading.Thread(target=camObj.capture)
    capture_thread.start()

'''      
    # Iterative capture execution
    for i in range(0, 3):
        print("Capture " + str(count+1))
        time.sleep(10)
        start = time.time()
        stream_video(start, count, client_socket, direct_path)
        count+=1

    # Capture execution
    start = time.time()
    
    cap_num = capture(start, count, client_socket, direct_path)

'''
    
