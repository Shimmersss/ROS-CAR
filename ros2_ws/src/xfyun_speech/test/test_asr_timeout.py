import websocket

from xfyun_speech.asr_node import XfyunAsrNode


class FakeSocket:
    def __init__(self, *, payload=None, raises_timeout=False):
        self.timeout = 5.0
        self.payload = payload
        self.raises_timeout = raises_timeout
        self.timeouts = []

    def gettimeout(self):
        return self.timeout

    def settimeout(self, value):
        self.timeout = value
        self.timeouts.append(value)

    def recv(self):
        if self.raises_timeout:
            raise websocket.WebSocketTimeoutException()
        return self.payload


class FakeAssembler:
    def consume(self, payload):
        del payload
        return 'text', True


def test_receive_poll_restores_timeout_after_empty_poll():
    socket = FakeSocket(raises_timeout=True)

    assert not XfyunAsrNode._receive_one(socket, FakeAssembler(), 0.001)
    assert socket.timeout == 5.0
    assert socket.timeouts == [0.001, 5.0]


def test_receive_poll_restores_timeout_after_result():
    socket = FakeSocket(payload='{}')

    assert XfyunAsrNode._receive_one(socket, FakeAssembler(), 0.001)
    assert socket.timeout == 5.0
    assert socket.timeouts == [0.001, 5.0]
