from lt_api import Link, load
import time


factory = load("/home/user/.local/liblt/liblt_zmq.so")

server = factory.make_server("tcp://*:5555")

while True:
    task = server.read()
    print(task)
    server.write("Opa!")

server.close()