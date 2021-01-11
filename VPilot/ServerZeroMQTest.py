
import json 
import zmq

from datetime import datetime, timedelta

class Server:
    def __init__(self, port = 8000):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.bind("tcp://127.0.0.1:" + str(8000))

        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.clientStarted = False
        self.lastSentMessage = datetime.now()
        self.rate = 20
        self.count = 0

    def checkRecvMessage(self):
        events = self.poller.poll(1)
        if (events != []):        
            message = self.socket.recv_string()
            try:
                json_dat = json.loads(message)
                if json_dat["start"] != None:
                    self.clientStarted = True
                if json_dat["stop"] != None:
                    self.clientStarted = False
            except Exception as e:
                print(e)

            print(message)

    def checkSendMessage(self):
        if ((datetime.now() - self.lastSentMessage) > (timedelta(seconds = 1 / self.rate))):
            self.count += 1
            msg = json.dumps({"count": self.count, "bbox2d": "testbbox"})
            self.socket.send_string(msg)
            self.lastSentMessage = datetime.now()


if __name__ == '__main__':
    server = Server(8000)
    while(True):
        server.checkRecvMessage()        

        if (server.clientStarted):
            server.checkSendMessage()
            # wait