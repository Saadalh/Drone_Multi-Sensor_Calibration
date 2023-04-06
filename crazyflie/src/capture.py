#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
#
#  AI-deck demo
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc., 51
#  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
#  Demo for showing streamed JPEG images from the AI-deck example.
#
#  By default this demo connects to the IP of the AI-deck example when in
#  Access point mode.
#
#  The demo works by opening a socket to the AI-deck, downloads a stream of
#  JPEG images and looks for start/end-of-frame for the streamed JPEG images.
#  Once an image has been fully downloaded it's rendered in the UI.
#
#  Note that the demo firmware is continously streaming JPEG files so a single
#  JPEG image is taken from the stream using the JPEG start-of-frame (0xFF 0xD8)
#  and the end-of-frame (0xFF 0xD9).

import argparse
import time
import socket,os,struct, time
import numpy as np
import threading
import cv2


# Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

class Webcam():
    def __init__(self, port, dir_path):
        self.cap = cv2.VideoCapture(port)
        self.dir_path = dir_path
        self.streaming = True
    
    def stream(self):
        while self.streaming == True:
            _, self.frame = self.cap.read()
            cv2.imshow("stream", self.frame)
            cv2.waitKey(1)
            
    def save_capture(self, repetition, station):
        self.repetition = repetition
        self.station = station
        cv2.imwrite(f"{self.dir_path}/capture_{self.repetition}{self.station}.jpg", self.frame)

    def stop_stream(self):
        self.streaming = False


class Camera():
    imgdata = None
    data_buffer = bytearray()
    stream = True

    def __init__(self, deck_ip, deck_port, dpath):
        self.start = time.time()
        self.dpath = dpath

        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print("Socket connected")

    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data

    def start_stream(self):
        print("Stream started.")
        count = 0
        while(self.stream):
            # First get the info
            packetInfoRaw = self.rx_bytes(4)
            #print(packetInfoRaw)
            [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
            #print("Length is {}".format(length))
            #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
            #print("Function is 0x{:02X}".format(function))

            imgHeader = self.rx_bytes(length - 2)
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
                    packetInfoRaw = self.rx_bytes(4)
                    [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                    #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                    chunk = self.rx_bytes(length - 2)
                    imgStream.extend(chunk)
                
                count = count + 1
                print(count)
                meanTimePerImage = (time.time()-self.start) / count
                print("{}".format(meanTimePerImage))
                print("{}".format(1/meanTimePerImage))

                if format == 0:
                    bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                    bayer_img.shape = (244, 324)
                    #cv2.imshow('Raw', bayer_img)
                    cv2.imwrite(f"{self.dpath}/img_{count:06d}_{time.time()}.jpg", bayer_img)
                    #cv2.waitKey(1)
                else:
                    with open("img.jpeg", "wb") as f:
                        f.write(imgStream)
                    nparr = np.frombuffer(imgStream, np.uint8)
                    decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                    cv2.imshow('JPEG', decoded)
                    cv2.waitKey(1)
        print("Stream Stopped.")
        cv2.destroyAllWindows()

    def stop_stream(self):
        self.stream = False

if __name__ == "__main__":
    dir_path = os.path.realpath(os.path.dirname(__file__))
    
    print("Starting thread...")
    time.sleep(1.5)
    
    camObj = Camera(args.n, args.p, f"{dir_path}/../captures/test")
    stream_thread = threading.Thread(target=camObj.start_stream)    
    stream_thread.start()

    time.sleep(10)

    print("Stopping program...")
    stop_thread = threading.Thread(target=camObj.stop_stream)
    stop_thread.start()
    
    #stream()