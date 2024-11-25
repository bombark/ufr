from ufr import Client

socket = Client("@new zmq:socket @host 127.0.0.1 @port 5000 @coder msgpack")
socket.put("is\n\n", 40, "felipe")
res = socket.get("^s\n")
socket.close()
print(res)