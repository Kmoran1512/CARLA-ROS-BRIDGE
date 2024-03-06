import numpy as np
import socket
import re

IP_ADDRESS = "192.168.56.30"
TARGET_PORT = 4242


class GazeReader:
    def __init__(self):
        self._server_connection()
        self.data = []

    def get_gaze(self):
        self._request_gaze_data()
        self._ignore_x_msgs(2)
        self._receive_gaze_data()

        if len(self.data) < 5:
            return (-1.0, -1.0)

        return separate_data(self.data)

    def _request_gaze_data(self):
        self._server.send(str.encode('<SET ID="ENABLE_SEND_BLINK" STATE="1" />\r\n'))
        self._server.send(str.encode('<SET ID="ENABLE_SEND_POG_FIX" STATE="1" />\r\n'))
        self._server.send(str.encode('<SET ID="ENABLE_SEND_DATA" STATE="1" />\r\n'))

    def _receive_gaze_data(self):
        raw_data = self._server.recv(1024)
        raw_data = bytes.decode(raw_data)

        raw_data = raw_data.split()[1:-1]

        raw_data = np.asarray(raw_data)
        raw_data = raw_data.reshape(-1, 1)

        numbers = re.compile(r"\d+(?:\.\d+)?")

        try:
            self.data = [float(numbers.findall(item[0])[0]) for item in raw_data]
        except IndexError:
            pass

    def _server_connection(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self._server:
            self._server.connect((IP_ADDRESS, TARGET_PORT))

    def _ignore_x_msgs(self, num):
        for _ in range(num):
            self._server.recv(1024)


def separate_data(data):
    """return x, y, v, d"""
    return (data[0], data[1])
