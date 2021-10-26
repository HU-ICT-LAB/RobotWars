"""Server side of websockets."""
import asyncio
import websockets


async def receive(websocket):
    """
    Wait for a message to come in, print it and respond.

    :param websocket: the websocket that needs to be used.
    """
    message = await websocket.recv()
    print(f"<<< {message}")
    await websocket.send("Server: Received successfully")


async def main():
    """Serve the websocket and make it run forever."""
    async with websockets.serve(receive, "localhost", 8765):
        await asyncio.Future()  # run forever


asyncio.run(main())
