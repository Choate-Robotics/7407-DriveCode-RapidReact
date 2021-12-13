import asyncio

import websockets
from websockets.exceptions import ConnectionClosedError

from lib.network.network_system import Network
from utils import logger

Network.ds_init()


async def send_status(websocket):
    logger.info("tcp socket opened", "[network]")

    try:
        while True:
            await websocket.send(Network.ds_get_status().serialize())
    except ConnectionClosedError:
        pass

    logger.info("tcp socket closed", "[network]")


async def main():
    async with websockets.serve(send_status, Network.local_websocket_ip, Network.local_websocket_port):
        await asyncio.Future()

asyncio.run(main())
