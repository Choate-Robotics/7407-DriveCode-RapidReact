import os
import subprocess
import sys
from multiprocessing.connection import Listener

from robotpy_toolkit_7407.utils import logger


class IntakeCameras:
    def __init__(self):
        # Start server
        args = [sys.executable, os.path.dirname(__file__) + "/intake_camera_server.py"]
        logger.info(f"starting {args}...")
        subprocess.Popen(args)
        logger.info("started!")

        # Bind to it
        # self.listener = Listener(('localhost', 6000))
        # self.conn = self.listener.accept()

    def read_camera_data(self):
        # self.conn.send("c")
        # if self.conn.poll(0.005):
        #     print(self.conn.recv())
        pass