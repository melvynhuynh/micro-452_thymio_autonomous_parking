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
TEST = False
PROXIMITY_THRESHOLD = 3300
def avoid(prox, ground):
    """Decide whether local navigation must take the wheels.

    prox    list of 7 numbers, horizontal proximity sensors (see robot.read).
    ground  list of 2 numbers, ground sensors, left then right.

    Gives back:
        None           nothing is in the way: motion control keeps the wheels.
        (left, right)  wheel speeds, in Thymio units, to steer away from the
                       obstacle. On the black border: (0, 0).

    Give back None only once the robot is clear of the obstacle, not the
    instant the sensors lose it. A variable kept in this file between two
    calls can help. The main loop then plans a new path around the cell of
    the obstacle.
    """
    # TODO (Laurent): real avoidance.
    return None

def obstacle_detected(prox):

    detected = [value > PROXIMITY_THRESHOLD for value in prox[:5]]
    obstacle = any(detected)

    return obstacle, detected

prox = [0, 1200, 3500, 1800, 0, 0, 0]

obstacle, sensors = obstacle_detected(prox)
print(obstacle)
print(sensors)

if obstacle:
    if sensors[0] or sensors[1]:
        print("Obstacle à gauche")

    if sensors[2]:
        print("Obstacle au centre")

    if sensors[3] or sensors[4]:
        print("Obstacle à droite")


if __name__ == "__main__":

    from tdmclient import ClientAsync

    with ClientAsync() as client:

        async def test_sensors():
            node = await client.wait_for_node()

            # Wait until sensor variables are available
            await node.wait_for_variables({
                "prox.horizontal",
                "prox.ground.delta"
            })

            print("Thymio connected! Reading sensors...")

            while True:
                prox = list(node.v.prox.horizontal)
                ground = list(node.v.prox.ground.delta)

                print(f"Proximity: {prox} | Ground: {ground}")

                await client.sleep(0.1)

        client.run_async_program(test_sensors)



# Small test module just in case
if (TEST):
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
