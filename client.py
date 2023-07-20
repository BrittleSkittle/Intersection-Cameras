import socket
import os
import tqdm
import time
import platform


HOST = "10.33.1.1"  # Node1-1 IP
PORT = 65432  # The port used by the server
SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096

def sendFile(filename):
    filesize = os.path.getsize(filename)
    s.send(f"{filename}{SEPARATOR}{filesize}".encode())
    progress = tqdm.tqdm(range(filesize), f"Sending {filename}", unit="B", unit_scale=True, unit_divisor=1024)
    with open(filename, "rb") as f:
        while True:
            # read the bytes from the file
            bytes_read = f.read(BUFFER_SIZE)
            if not bytes_read:
                # file transmitting is done
                break
            # we use sendall to assure transimission in busy networks
            s.sendall(bytes_read)
            # update the progress bar
            progress.update(len(bytes_read))
    progress.close()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    Node = platform.node().split('-')[0][4]
    print("Currently on Node "+Node+".\n")    
    s.connect((HOST, PORT))
    filename = "NA"
    if Node == "1":
        print("Currently on Node 1, not sending files.")
        filename = "q"
    while filename != "q":

        print("Type the name of the file you would like to send to Node1-1 in this directory, or type 'all' to send all relevant files. \n")
        filename = input("")
        if filename == "all":
            try:
                sendFile("ids"+str(Node)+".pkl")
                time.sleep(1)
            except Exception as e:
                print(str(filename)+"not sent due to Exception: "+str(e)+".\n")
            try:
                sendFile("corners3D"+str(Node)+".pkl")
                time.sleep(1)
            except Exception as e:
                print(str(filename)+"not sent due to Exception: "+str(e)+".\n")
            try:
                sendFile("color_source"+str(Node)+".pkl")
                time.sleep(1)
            except Exception as e:
                print(str(filename)+"not sent due to Exception: "+str(e)+".\n")
            try:
                sendFile("texcoords"+str(Node)+".pkl")
                time.sleep(1)
            except Exception as e:
                print(str(filename)+"not sent due to Exception: "+str(e)+".\n")
            try:
                sendFile("verts"+str(Node)+".pkl")
                time.sleep(1)
            except Exception as e:
                print(str(filename)+"not sent due to Exception: "+str(e)+".\n")
            filename = "q"
            break
        else:
            sendFile(filename)
        
s.close()
