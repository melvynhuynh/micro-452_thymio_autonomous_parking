
"""
=========================================================
            LOCAL NAVIGATION -- local_nav.py
=========================================================

Inputs:
    prox:      7 horizontal proximity sensor values
    ground:    2 ground sensor values (left, right)
    threshold: horizontal obstacle detection threshold

Outputs:
    None              Local navigation does not take control.
    (left, right)     Local navigation overrides motor commands.
    (0, 0)            Black border detected.

Author: Laurent Duret
Date: 17 November 2026
"""

# =======================================================
# PARAMETERS
# =======================================================

FOLLOW_THRESHOLD = 3000

# TODO: Calibrate this threshold on the actual parking surface. !!!
PARKING_THRESHOLD = 4600

FORWARD_SPEED = 80
TURN_SPEED = 70
TURN_REVERSE_SPEED = 30

CLEAR_LIMIT = 5

GROUND_BLACK_THRESHOLD = 400  #full black = less than 200


# =======================================================
# INTERNAL STATE
# =======================================================

_avoiding = False
_clear_count = 0
_direction = "right"


# =======================================================
# HELPER FUNCTIONS
# =======================================================

def obstacle_detected(prox, threshold):
    """Detect obstacles using the five front sensors."""

    sensors = [value > threshold for value in prox[:5]]
    return any(sensors), sensors


def choose_direction(prox, threshold):
    """Choose the direction with the least detected obstacle."""

    left_strength = sum(
        max(0, value - threshold) for value in prox[0:2]
    )

    right_strength = sum(
        max(0, value - threshold) for value in prox[3:5]
    )

    if left_strength > right_strength:
        return "right"
    elif right_strength > left_strength:
        return "left"
    else:
        return "right"


def motor_command(direction):
    """Return motor speeds according to the desired direction."""

    if direction == "right":
        return FORWARD_SPEED, -TURN_REVERSE_SPEED

    elif direction == "left":
        return -TURN_REVERSE_SPEED, FORWARD_SPEED

    return FORWARD_SPEED, FORWARD_SPEED


# =======================================================
# MAIN LOCAL NAVIGATION INTERFACE
# =======================================================

def avoid(prox, ground, threshold):
    """
    Decide whether local navigation must override motor control.

    Args:
        prox: 7 horizontal proximity sensor values.
        ground: 2 ground sensor values.
        threshold: horizontal obstacle detection threshold.

    Returns:
        None: no obstacle, normal motion can continue.
        (left, right): override motor commands.
        (0, 0): black border detected.
    """

    global _avoiding, _clear_count, _direction

    # 1. Ground detection (black border)
    if GROUND_BLACK_THRESHOLD is not None:
        if any(value < GROUND_BLACK_THRESHOLD for value in ground):
            return (0, 0)

    # 2. Horizontal obstacle detection
    obstacle, sensors = obstacle_detected(prox, threshold)

    # 3. Start obstacle avoidance
    if not _avoiding and obstacle:
        _avoiding = True
        _clear_count = 0
        _direction = choose_direction(prox, threshold)

    # 4. Update avoidance state
    if _avoiding:

        if obstacle:
            _clear_count = 0
            _direction = choose_direction(prox, threshold)

        else:
            _clear_count += 1

        # Resume normal navigation only after
        # several consecutive clear readings.
        if _clear_count >= CLEAR_LIMIT:
            _avoiding = False
            _clear_count = 0
            return None

        # 5. Override motor commands
        return motor_command(_direction)

    return None


def reset():
    """Reset internal navigation state."""

    global _avoiding, _clear_count, _direction

    _avoiding = False
    _clear_count = 0
    _direction = "right"


# =======================================================
# TEST MODULE
# =======================================================

from tdmclient import ClientAsync


if __name__ == "__main__":

    # Select the threshold to test here.
    TEST_THRESHOLD = PARKING_THRESHOLD

    with ClientAsync() as client:

        async def test_sensors():

            with await client.lock() as node:

                await node.wait_for_variables({
                    "prox.horizontal",
                    "prox.ground.delta",
                    "motor.left.target",
                    "motor.right.target"
                })

                print("Thymio connected!")
                print("Starting obstacle avoidance test...")
                print(f"Detection threshold: {TEST_THRESHOLD}")

                reset()

                try:
                    while True:

                        prox = list(node.v.prox.horizontal)
                        ground = list(node.v.prox.ground.delta)

                        # Simulate global navigation commanding forward.
                        left, right = FORWARD_SPEED, FORWARD_SPEED

                        # Local navigation override
                        override = avoid(prox, ground, TEST_THRESHOLD)

                        if override is not None:
                            left, right = override
                            state = "LOCAL NAV OVERRIDE"
                        else:
                            state = "GLOBAL NAV"

                        node.v.motor.left.target = left
                        node.v.motor.right.target = right
                        node.flush()

                        print(
                            f"{state} | "
                            f"Prox: {prox[:5]} | "
                            f"Motors: ({left}, {right}) | "
                            f"Threshold: {TEST_THRESHOLD}"
                        )

                        await client.sleep(0.1)

                finally:
                    node.v.motor.left.target = 0
                    node.v.motor.right.target = 0
                    node.flush()
                    print("Motors stopped.")

        client.run_async_program(test_sensors)