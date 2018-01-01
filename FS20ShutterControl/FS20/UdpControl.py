#!/usr/bin/env python
     
import socket
import sys
     
HOST = '192.168.178.71'
PORT = 5000

# open socket connection 
try:
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error as msg:
  sys.stderr.write("[ERROR] %s\n" % msg)
  sys.exit(1)
     
try:
  sock.connect((HOST, PORT))
except socket.error as msg:
  sys.stderr.write("[ERROR] %s\n" % msg)
  sys.exit(2)


# get commands from terminal and send to Arduino
# terminate with 'exit'
while True:
  line = input("enter cmd: ")

  if(line=="exit"):
    sock.send(bytes("exit",'UTF-8'))
    sys.stdout.write("exiting...\n")
    break

  sock.send(bytes(line,'UTF-8'))
  data = sock.recv(1024)
  answer = data.decode('UTF-8')

  sys.stdout.write("response:  "+answer+"\n")

  try:
    seconds = int(answer[2:])
    h = seconds // 3600
    m = (seconds-h*3600) // 60
    s = seconds-h*3600-m*60
    sys.stdout.write("time: {}:{}:{}\n".format(h,m,s))
  except:
    sys.stdout.write("no time\n")
  
sock.close()
     

