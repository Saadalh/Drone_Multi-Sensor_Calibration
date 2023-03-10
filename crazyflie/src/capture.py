import cv2
import time
import argparse
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

# WiFi connection method
# If it didn't work outdent lines 31,32,33, and change main accordingly.
def connect_wifi(deck_ip, deck_port):
    print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((deck_ip, deck_port))
    print("Socket connected")
    return client_socket

# Image raw-pack information setup
def rx_bytes(size, client_socket):
  data = bytearray()
  while len(data) < size:
    data.extend(client_socket.recv(size-len(data))) # similar to append()
  return data

# Image acquisition method
def capture(start, count, client_socket, dir_path):
    
    packetInfoRaw = rx_bytes(4, client_socket)
    #print(packetInfoRaw)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
    #print("Length is {}".format(length))
    #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
    #print("Function is 0x{:02X}".format(function))

    imgHeader = rx_bytes((length - 2), client_socket)
    #print(imgHeader)
    #print("Length of data is {}".format(len(imgHeader)))
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic == 0xBC:
      #print("Magic is good")
      #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
      #print("Image format is {}".format(format))
      #print("Image size is {} bytes".format(size))

      # Now we start rx the image, this will be split up in packages of some size
      imgStream = bytearray()

      while len(imgStream) < size:
          packetInfoRaw = rx_bytes(4, client_socket)
          [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
          #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
          chunk = rx_bytes((length - 2), client_socket)
          imgStream.extend(chunk)
    
      count = count + 1
      meanTimePerImage = (time.time()-start) / count
      #print("{}".format(meanTimePerImage))
      #print("{}".format(1/meanTimePerImage))

      filename = dir_path+f"/captures/img_{count:06d}_"+str(start)+".jpg"
      if format == 0:
          bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
          bayer_img.shape = (244, 324)
          #cv2.imshow('Raw', bayer_img)
          if args.unsave:
              if not cv2.imwrite(filename, bayer_img):
                raise Exception("Could not save capture image")
          cv2.waitKey(1)
      else:
          with open("img.jpeg", "wb") as f:
              f.write(imgStream)
          nparr = np.frombuffer(imgStream, np.uint8)
          decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
          cv2.imshow('JPEG', decoded)
          cv2.waitKey(1)
    cap_num = "00000" + str(count)
    return cap_num

if __name__ == '__main__':
    # Initialize the low-level cflib drivers
    cflib.crtp.init_drivers()
    client_socket = connect_wifi(deck_ip, deck_port)

    # Capture execution
    start = time.time()
    count = 0
    cap_num = capture(start, count, client_socket)

'''
    # Iterative capture execution
    for i in range(0, 30):
        print("Capture " + str(count+1))
        time.sleep(3)
        start = time.time()
        cap_num = capture(start, count)
        count+=1
'''