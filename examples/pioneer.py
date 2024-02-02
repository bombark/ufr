import ufr

robot = ufr.Robot()

robot.install_source("../ufr-gtw-zmq")
robot.install_source("../ufr-cc-msgpack")
robot.install_source("../vri-base-pioneer-webots")
robot.install_source("../ufr-app-filesystem")

robot.make_topic("lidar", "@new zmq:topic @host 127.0.0.1 @port 6001 @coder msgpack:obj")
robot.make_topic("encoder", "@new zmq:topic @host 127.0.0.1 @port 6004 @coder msgpack:obj")

# robot.make_control("@new zmq:socket @host 127.0.0.1 @port 4000")

robot.generate_bash_source()