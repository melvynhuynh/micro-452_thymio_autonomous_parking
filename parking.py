"""
=========================================================
            LOCAL NAVIGATION -- parking.py 
=========================================================

Inputs:


Outputs:


Author: Laurent Duret
Date: 11 November 2026
=========================================================
"""
from tdmclient import ClientAsync

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

with ClientAsync() as client:

    async def program():
        with await client.lock() as node:

            try:
                print("Motors start")
                await node.set_variables(motors(100, 100))

                await client.sleep(5)

            finally:
                await node.set_variables(motors(0, 0))
                print("Motors stop")

    client.run_async_program(program)
