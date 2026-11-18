#!/bin/bash
# con.sh "<console line>": send one line to the dome ESP32's console via the dome Pi, print the reply.
timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=8 davids@10.0.19.163 "timeout 8 python3 - $(printf '%q' "$1")" <<'PY'
import os,sys,time,select
p="/dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_123456-if00"
fd=os.open(p,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
os.write(fd,(sys.argv[1]+"\r\n").encode()); buf=b""; t=time.time()
while time.time()-t<2.0:
    r,_,_=select.select([fd],[],[],0.2)
    if r:
        try: buf+=os.read(fd,4096)
        except BlockingIOError: pass
print(buf.decode("utf8","replace").strip())
PY
