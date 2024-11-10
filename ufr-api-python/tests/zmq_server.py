from ufr import Server

socket = Server("@new zmq:socket @host 127.0.0.1 @port 5000 @coder msgpack")
# while (1):
res = socket.get("^i\n")
print(res)
socket.put("s\n\n", "OK")
socket.close()