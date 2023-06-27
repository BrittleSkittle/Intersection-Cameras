#sudo ufw allow 65432
#must be done on both server and client

import socket
from socket import gethostbyname
import os
import tqdm

BUFFER_SIZE = 4096
SEPARATOR = "<SEPARATOR>"
HOST = "10.33.1.1"  # Node1-1 IP
#HOST = gethostbyname( '0.0.0.0' )
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
stop = False
while not stop:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Server is currently running...\n Now run client.py on a different node and type the file to transfer.\n Press 'q' to stop.")
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        received = conn.recv(BUFFER_SIZE).decode()
        filename, filesize = received.split(SEPARATOR)
        filename = os.path.basename(filename)
        filesize = int(filesize)
        progress = tqdm.tqdm(range(filesize), f"Receiving {filename}", unit="B", unit_scale=True, unit_divisor=1024)
        with open(filename, "wb") as f:
            while True:
                # read 1024 bytes from the socket (receive)
                bytes_read = conn.recv(BUFFER_SIZE)
                if not bytes_read:    
                    # nothing is received
                    # file transmitting is done
                    break
                # write to the file the bytes we just received
                f.write(bytes_read)
                # update the progress bar
                progress.update(len(bytes_read))
    if keyboard.is_pressed('q'):
        stop = True

conn.close() 
s.close()   