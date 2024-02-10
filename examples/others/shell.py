import ufr

ctrl = ufr.Link("@new zmq:socket @port 4000 @coder msgpack:obj")
ctrl.connect()

while True:
    line_raw = input("root# ")
    line = line_raw.split(' ')
    if line[0] != '':
        cmd = line[0]
        ctrl.putln(cmd)
        ctrl.recv()
        nome = ctrl.get("s")
        print(nome)

ctrl.close()