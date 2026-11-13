"""
=========================================================
            LOCAL NAVIGATION -- local_nav.py
=========================================================

Inputs:


Outputs:


Author: Laurent Duret
Date: 11 November 2026
=========================================================
"""

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


from tdmclient import ClientAsync


# Detection parameters
PROX_THRESHOLD = 3000

# Motor speeds 
FORWARD_SPEED = 80
TURN_SPEED = 70
TURN_REVERSE_SPEED = 30

# Number of consecutive clear readings before resuming
CLEAR_LIMIT = 5


def obstacle_detected(prox):
    """Detect obstacles using the five front sensors."""

    sensors = [value > PROX_THRESHOLD for value in prox[:5]]
    obstacle = any(sensors)

    return obstacle, sensors


def choose_direction(prox):
    """Choose the direction with the least detected obstacle."""

    left_strength = sum(max(0, value - PROX_THRESHOLD)
                        for value in prox[0:2])

    right_strength = sum(max(0, value - PROX_THRESHOLD)
                         for value in prox[3:5])

    if left_strength > right_strength:
        return "right"
    elif right_strength > left_strength:
        return "left"
    else:
        return "right" #right by default

def motor_command(direction):
    """Return motor speeds according to the desired direction."""

    if direction == "right":
        return FORWARD_SPEED, -TURN_REVERSE_SPEED

    elif direction == "left":
        return -TURN_REVERSE_SPEED, FORWARD_SPEED

    else:
        return FORWARD_SPEED, FORWARD_SPEED

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

from tdmclient import ClientAsync

# Test module
if __name__ == "__main__":

    with ClientAsync() as client:

        async def test_sensors():

            node = await client.wait_for_node()

            await node.wait_for_variables({
                "prox.horizontal",
                "prox.ground.delta"
            })

            print("Thymio connected!")
            print("Starting obstacle avoidance test...")

            avoiding = False
            clear_count = 0
            direction = "right"

            try:
                while True:

                    prox = list(node.v.prox.horizontal)
                    ground = list(node.v.prox.ground.delta)

                    obstacle, sensors = obstacle_detected(prox)

                    if not avoiding and obstacle:
                        avoiding = True
                        clear_count = 0
                        direction = choose_direction(prox)

                    if avoiding:
                        if obstacle:
                            clear_count = 0
                            direction = choose_direction(prox)
                        else:
                            clear_count += 1

                        if clear_count >= CLEAR_LIMIT:
                            avoiding = False
                            clear_count = 0

                    if avoiding:
                        left, right = motor_command(direction)
                        state = f"AVOIDING ({direction})"
                    else:
                        left, right = motor_command("forward")
                        state = "FORWARD"

                    node.v.motor.left.target = left
                    node.v.motor.right.target = right
                    node.flush()

                    print(f"{state} | Prox: {prox[:5]} | Motors: ({left}, {right})")

                    await client.sleep(0.1)

            finally:
                node.v.motor.left.target = 0
                node.v.motor.right.target = 0
                node.flush()
                print("Motors stopped.")

        client.run_async_program(test_sensors)