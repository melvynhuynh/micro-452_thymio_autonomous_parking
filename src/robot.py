"""Link with the Thymio. Owner: Antoine.

The only file that talks to the robot: reads its sensors, sets its wheels.

It uses the tdmclient library. Thymio Suite must be open and the Thymio
plugged in (cable or dongle) before connect() is called.

Always stop the wheels when a program ends, even after an error:

    robot.connect()
    try:
        ...
    finally:
        robot.set_speed(0, 0)
"""

client = None   # the link with Thymio Suite, set by connect()
node = None     # the Thymio itself, set by connect()
READY = 4       # status of a Thymio this program holds (tdmclient's number)

# The variables of the Thymio that read() needs.
VARIABLES = {"prox.horizontal", "prox.ground.delta",
             "motor.left.speed", "motor.right.speed"}


def connect():
    """Connect to the Thymio. Call it once, before anything else."""
    global client, node
    # Imported here, so that the simulations run without the library.
    import threading
    from tdmclient import ClientAsync

    client = ClientAsync()
    # tdmclient reads the robot in a thread that never ends by itself, so a
    # program would hang when it finishes. This closes the link first. (Not
    # atexit: Python waits for the threads before running atexit.)
    threading._register_atexit(client.disconnect)
    node = client.aw(client.wait_for_node())        # waits until a Thymio is there
    client.aw(node.lock())                          # nobody else drives it
    client.aw(node.wait_for_variables(VARIABLES))   # first values received


def read():
    """Read the sensors of the Thymio.

    Gives back three values:
        prox          list of 7 numbers, horizontal proximity sensors, in the
                      Thymio order: front left, front centre-left, front
                      centre, front centre-right, front right, back left,
                      back right. 0 = nothing, larger = closer.
        ground        list of 2 numbers, ground sensors, left then right.
                      High on white paper, low on the black border.
        wheel_speeds  (left, right) speeds measured by the Thymio, in Thymio
                      units.
    """
    client.process_waiting_messages()   # take in the latest values sent

    # While this program holds the Thymio, the device manager says it is
    # "ready" (4). Anything else means the link was lost, for example the USB
    # cable came out: commands no longer reach the robot and the values below
    # are frozen. Stop rather than drive blind.
    if node.status != READY:
        raise RuntimeError("Lost the link with the Thymio (USB cable "
                           "unplugged?). Plug it back and start the program "
                           "again.")

    prox = list(node.var["prox.horizontal"])
    ground = list(node.var["prox.ground.delta"])
    wheel_speeds = (node.var["motor.left.speed"][0],
                    node.var["motor.right.speed"][0])
    return prox, ground, wheel_speeds


def set_speed(left, right):
    """Set the speed of the two wheels, in Thymio units. (0, 0) stops."""
    targets = {"motor.left.target": [round(left)],
               "motor.right.target": [round(right)]}
    # Sent at once, without waiting for the Thymio to answer: waiting takes
    # about 0.1 s, the whole period of the main loop (measured 2026-11-17).
    node.send_set_variables(targets)
