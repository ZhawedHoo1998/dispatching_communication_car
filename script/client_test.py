import socket
import time
soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
soc.connect(('192.168.1.104', 8088))
send_data = "{'goal':[0, 0, 0],'mission_state':'go'}"
while True:
    soc.send(send_data.encode('utf-8'))
    print(soc.recv(1024).decode('utf-8'))
soc.close()
