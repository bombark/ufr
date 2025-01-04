from ufr import Publisher

topic = Publisher("@new zmq:topic @coder msgpack")
topic.put("iifff\n", 10,20,30.0,12.5,26.4)
topic.close()