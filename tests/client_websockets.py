#!/usr/bin/env python

import asyncio
import websockets

#The url the websocket will attempt to connect to.
#Example: ""ws://localhost:8765""
websocket_url = "ws://localhost:8765"

#Calling this function starts a very basic routine which waits for messages and sends 'world' back whenever it receives "hello"
async def run():
        async with websockets.connect(websocket_url) as websocket:

                while(True):
                        #Wait for input
                        #Recv return an object of type 'websockets.typing.Data'
                        #https://websockets.readthedocs.io/en/stable/reference/types.html#websockets.typing.Data
                        msg = await websockets.recv()

                        #If the message is 'hello', send 'world' back
                        if(msg == "hello"):
                                await(websocket.send("world"))
                                print("'hello world' send back to server")


asyncio.run(run())

