import asyncio

import websockets
from robotpy_toolkit_7407.network.network_system import Network
from robotpy_toolkit_7407.utils import logger
from websockets.exceptions import ConnectionClosedError

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
