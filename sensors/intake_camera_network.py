import socket


class UDPServer:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind(('', 5804))

    def send_packet(self, data: bytes):
        self._socket.sendto(data, ("<broadcast>", 5804))


class UDPClient:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind(('', 5804))

    def recv_packet(self) -> bytes:
        return self._socket.recvfrom(500)[0]
