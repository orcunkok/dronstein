"""The link to Lucky's flight controller (GcsLink.cs), the way a GCS talks to ArduPilot SITL: listen on UDP 14550, take
the FC's state from whatever arrives, and send the sticks and the latest command back to the sender, as one JSON line
each way. Either side can start first; they find each other. Lucky converts its frames on its side: this sees NED and
ArduPilot's modes."""
import json
import socket
import time

PORT = 14550
TIMEOUT = 1.0  # s without the FC's state: not connected


class LuckyLink:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", PORT))
        self.sock.setblocking(False)
        self.fc = None
        self.last = -TIMEOUT

    def step(self, state):
        try:
            while True:
                data, self.fc = self.sock.recvfrom(4096)
                m = json.loads(data)
                state.armed, state.mode, state.landed = m["armed"], m["mode"], m["landed"]
                state.pos, state.vel, state.att = tuple(m["pos"]), tuple(m["vel"]), tuple(m["att"])
                state.gripper, state.ack = m["gripper"], m["ack"]
                self.last = time.monotonic()
        except BlockingIOError:
            pass
        state.connected = time.monotonic() - self.last < TIMEOUT
        if state.connected:
            out = {"rc": state.rc, "seq": state.seq, "cmd": state.cmd, "value": state.value}
            self.sock.sendto(json.dumps(out).encode(), self.fc)
