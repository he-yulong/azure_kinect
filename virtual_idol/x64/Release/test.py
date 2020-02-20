import socket
import json

ip_port=('',8888)
BUFSIZE=1024 * 1024
sock_server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock_server.bind(ip_port)

output = open('./test_data.txt', 'w')

while True:
    msg,addr=sock_server.recvfrom(BUFSIZE)
    print('recv:',msg,addr)
    # sock_server.sendto(msg.upper(),addr)
    output.write(msg + '\n')

output.close()