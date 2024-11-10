from ufr import Client

socket = Client("@new zmq:socket @host 127.0.0.1 @port 5000 @coder msgpack")
socket.put("i\n\n", 40)
res = socket.get("^s\n")
socket.close()
print(res)