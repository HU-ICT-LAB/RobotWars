"""Client side of websocket."""
import asyncio
import websockets


uri = "ws://localhost:8765"


async def send():
    """Send message to a websocket."""
    async with websockets.connect(uri) as websocket:
        while True:
            out_message = input("Message: ")
            await websocket.send(out_message)
            in_message = await websocket.recv()
            print(f"<<< {in_message}")


asyncio.run(send())
