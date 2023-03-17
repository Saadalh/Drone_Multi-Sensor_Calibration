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
            print(f"size is: {len(data)}")
            data.extend(self.client_socket.recv(size-len(data))) # similar to append()
        return data

    # Continuous frames acquisition method
    def start_stream(self, count, dir_path):
        print("Stream Started")
        time.sleep(0.1)
        while(self.stream):
            packetInfoRaw = self.rx_bytes(4)

            #print("unpacking PacketInfo")
            [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

            #print("GETTINGHEADER")
            imgHeader = self.rx_bytes((length - 2))

            #print("Unpacking Header!")
            [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

            if magic == 0xBC:

            # Now we start rx the image, this will be split up in packages of some size
                #print("STUCK")
                imgStream = bytearray() # Getting stuck here
                
                while len(imgStream) < size:
                    packetInfoRaw = self.rx_bytes(4)
                    [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                    chunk = self.rx_bytes((length - 2))
                    imgStream.extend(chunk)
                
                count = count + 1
                filename = dir_path+f"/img_{count:06d}_"+str(time.time())+".jpg"

                if format == 0:
                    bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                    bayer_img.shape = (244, 324)
                    if 1: # Add your frame saving condition here
                        #print("Saving Capture...")
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
        print("Stream Stopped")

    # On-demand single frame capturing method
    def capture(self):
        self.save = True

    # Test capturing method
    def capture_test(self):
        for x in range(0, 4):
            time.sleep(3)
            print("CAPTURE!")
            self.save = True

    def stop_stream(self):
        self.stream = False

if __name__ == '__main__':
    direct_path = os.path.realpath(os.path.dirname(__file__))
    # Initialize the low-level cflib drivers
    cflib.crtp.init_drivers()
    camObj = Camera(deck_ip, deck_port)

    stream_start_thread = threading.Thread(target=camObj.start_stream, args=( 0, f"{direct_path}/../captures"))
    capture_test_thread = threading.Thread(target=camObj.capture_test)
    stop_thread = threading.Thread(target=camObj.stop_stream)

    stream_start_thread.start()
    
    time.sleep(60)
    
    stop_thread.start()
    stop_thread.join()
    stream_start_thread.join()

    print("Done")