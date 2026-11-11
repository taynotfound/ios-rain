#!/usr/bin/env python3
"""Receive the phone's live kernel log (UDP 5140 over USB Ethernet, patch 0988) and append it to a file.
   build-rain/usblog.py LOGFILE        (the PC side needs 192.168.7.1/24 on the phone's interface)"""
import socket, sys, time
out = open(sys.argv[1], 'ab', buffering=0)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 5140))
out.write(f'--- listening {time.strftime("%H:%M:%S")} ---\n'.encode())
while True:
    data, _ = s.recvfrom(65536)
    out.write(data)
