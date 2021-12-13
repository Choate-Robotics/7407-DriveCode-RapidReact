from lib.subsystem import Subsystem
from lib.network.objects import RobotStatusPacket
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

    ## TODOs
    TODO Make sure pickle maintains the subsystem data structs (and not just NetworkObjects)
    TODO Does TCP allow packet response codes?
    """

    subsystems: list[Subsystem]

    @staticmethod
    def robot_init(subsystems: list[Subsystem]):
        logger.info("initializing network on robot", "[network]")
        Network.subsystems = subsystems
        logger.info("initialization complete", "[network]")

    @staticmethod
    def robot_get_status() -> RobotStatusPacket:
        subsystem_status = []

        for s in Network.subsystems:
            subsystem_status.append(s.get_network_object())

        return RobotStatusPacket(
            subsystems=subsystem_status
        )
