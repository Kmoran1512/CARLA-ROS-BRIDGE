#!/usr/bin/env python3
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
        rxdat = self._server.recv(1024)
        rxdat = bytes.decode(rxdat)

        rxdat = rxdat.split()

        rxdat = rxdat[1:-1]

        rxdat = np.asarray(rxdat)
        rxdat = rxdat.reshape(-1, 1)

        numbers = re.compile(r"\d+(?:\.\d+)?")

        self.data = []
        try:
            for i in range(len(rxdat)):
                temp = numbers.findall(rxdat[i][0])
                temp = float(temp[0])
                self.data.append(temp)
        except IndexError:
            pass

    def _server_connection(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.connect((IP_ADDRESS, TARGET_PORT))

    def _ignore_x_msgs(self, num):
        for _ in range(num):
            self._server.recv(1024)


def separate_data(data):
    """return x, y, v, d"""
    return (data[0], data[1])
