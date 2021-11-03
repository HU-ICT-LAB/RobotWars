"""Connects to the server and receives a file."""
import socket

s = socket.socket()
s.connect(("localhost", 9999))

file = open("incoming_file.txt", "wb")
data = s.recv(1024)
print("Received data: " + str(data))
file.write(data)

file.close()
s.close()
