import pickle
import socket

import websockets

from lib.subsystem import Subsystem
from lib.network.objects import RobotStatusPacket, NetworkObject
from utils import logger


class Network:
    """
    ### SCHEMA:

    # Robot status:
    - UDP status packet is sent by the robot to python on the DS laptop (port 5800)
       - Pickled string of the RobotStatusPacket struct
    - Python backend on DS is responsible for unpickling, validation, and converting to JSON for the frontend

    # DS to Robot communication
    - Python backend responsible for converting the actions into packets
    - TCP packets on port 5801
    - Pickled string
    - Each distinct command will have a defined struct
    - Response codes??
    """

    subsystems: list[Subsystem]

    udp_ip: str = "127.0.0.1"
    udp_port: int = 5800
    udp_socket: socket.socket
    udp_max_packet_size: int = 32768

    local_websocket_ip: str = "localhost"
    local_websocket_port: int = 8765

    @classmethod
    def robot_init(cls, subsystems: list[Subsystem]):
        logger.info("initializing network on robot", "[network]")

        cls.subsystems = subsystems

        logger.info("initializing udp", "[network]")
        cls.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        logger.info("initialization complete", "[network]")

    @classmethod
    def ds_init(cls):
        logger.info("initializing network on driver station", "[network]")

        logger.info("initializing udp", "[network]")
        cls.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.udp_socket.bind((cls.udp_ip, cls.udp_port))

        logger.info("initialization complete", "[network]")

    @classmethod
    def robot_send_status(cls):
        cls._udp_send_network_obj(cls._robot_get_status())

    @classmethod
    def ds_get_status(cls) -> RobotStatusPacket:
        while True:
            status = cls._udp_receive_network_obj()
            if isinstance(status, RobotStatusPacket):
                return status

    @classmethod
    def _robot_get_status(cls) -> RobotStatusPacket:
        subsystem_status = []

        for s in cls.subsystems:
            subsystem_status.append(s.get_network_object())

        return RobotStatusPacket(
            subsystems=subsystem_status
        )

    @classmethod
    def _udp_send_network_obj(cls, obj: NetworkObject):
        data = pickle.dumps(obj)
        if len(data) > cls.udp_max_packet_size:
            logger.warn(f"Network object {obj} exceeded maximum packet size and was not sent")
            return
        cls.udp_socket.sendto(data, (cls.udp_ip, cls.udp_port))

    @classmethod
    def _udp_receive_network_obj(cls) -> NetworkObject:
        data, _ = cls.udp_socket.recvfrom(cls.udp_max_packet_size)
        obj: NetworkObject = pickle.loads(data)
        return obj
