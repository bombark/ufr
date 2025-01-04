from ufr import Subscriber

topic = Subscriber("@new zmq:topic @coder msgpack @debug 4")
res = topic.get("^iifff")
print(res)
topic.close()
