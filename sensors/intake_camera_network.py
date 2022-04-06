import socket


PORT = 37021


class UDPServer:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind(('', PORT))

    def send_packet(self, data: bytes):
        self._socket.sendto(data, ("<broadcast>", PORT))


class UDPClient:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind(('', PORT))

    def recv_packet(self) -> bytes:
        return self._socket.recvfrom(500)[0]
