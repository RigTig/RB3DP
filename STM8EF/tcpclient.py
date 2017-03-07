#!/usr/bin/env python2

import socket
import sys
import time

if len(sys.argv) < 2:
    print('Usage %s <file1> ... [fileN]' % (sys.argv[0]))
    sys.exit()


HOST, PORT = "172.16.0.10", 8088


# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to server 
sock.connect((HOST, PORT))
    

def upload(path): 
    with open(path) as source:
        for line in source.readlines():
            time.sleep(0.2)        
            line = line.strip()
            if not line: continue
            if len(line) > 64:
                raise 'Line is too long: %s' % (line)
            print('\n\rsending: ' + line)
            sock.sendall(line + "\n\r")
            chin = ''
            response_buffer = []
            while chin <> '>':
	      response_buffer.append(chin)
	      chin = sock.recv(1)
            response = ''.join(response_buffer)
            sys.stdout.write(response)

for path in sys.argv[1:]:
    print('Uploading %s' % path)
    upload(path)
    
sock.close()