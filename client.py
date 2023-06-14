import socket
import os
import tqdm
print('MAKE SURE SERVER.PY IS RUNNING ON MAIN NODE')
filename = input("Type the name of the file you would like to send to Node1-1 in this directory.\n")

HOST = "10.33.1.1"  # Node1-1 IP
PORT = 65432  # The port used by the server
filesize = os.path.getsize(filename)
SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(f"{filename}{SEPARATOR}{filesize}".encode())
    progress = tqdm.tqdm(range(filesize), f"Sending {filename}", unit="B", unit_scale=True, unit_divisor=1024)
    with open(filename, "rb") as f:
        while True:
            # read the bytes from the file
            bytes_read = f.read(BUFFER_SIZE)
            if not bytes_read:
                # file transmitting is done
                break
            # we use sendall to assure transimission in 
            # busy networks
            s.sendall(bytes_read)
            # update the progress bar
            progress.update(len(bytes_read))
s.close()
