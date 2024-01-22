from lt_api import Link
import time


topic = Link(new="zmq:topic", host="127.0.0.1", port=5000, encoder="msgpack:obj")
topic.start_publisher()
topic.putln(10,20,30)
topic.close()