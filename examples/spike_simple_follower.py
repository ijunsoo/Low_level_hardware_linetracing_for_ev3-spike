# Simple line follower for a LEGO SPIKE Prime hub running Pybricks.
#
# Robot: left motor on port E, right motor on port F, and three EV3 Color
# Sensors in a row in front of the wheels: C = left, A = middle, B = right.
# EV3 sensors on a SPIKE hub are read with PUPDevice; mode 0 gives the
# reflected light in percent.
#
# Simulator (from the repository root):
#   python sim/run_sim.py examples/spike_simple_follower.py --profile spike_3s_geo2 --track main --no-plot

from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# The right motor is mounted mirrored, so it turns counterclockwise to drive forward.
left_motor = Motor(Port.E)
right_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)

left_sensor = PUPDevice(Port.C)
middle_sensor = PUPDevice(Port.A)
right_sensor = PUPDevice(Port.B)

# Reflection each sensor reads on the white floor and on the black line.
# Measure your own robot (print(sensor.read(0)[0])) and put the numbers here.
LEFT_WHITE, LEFT_BLACK = 34, 6
MIDDLE_WHITE, MIDDLE_BLACK = 21, 3
RIGHT_WHITE, RIGHT_BLACK = 22, 3

SPEED = 40        # forward power in % (motor duty)
GAIN = 0.4        # steering per point of brightness difference
SEARCH_TURN = 60  # steering while no sensor sees the line


def brightness(sensor, white, black):
    # Scale the reading to 0 (black line) ... 100 (white floor).
    reflection = sensor.read(0)[0]
    return (reflection - black) * 100 / (white - black)


line_side = 1  # where the line was seen last: 1 = right, -1 = left

while True:
    left = brightness(left_sensor, LEFT_WHITE, LEFT_BLACK)
    middle = brightness(middle_sensor, MIDDLE_WHITE, MIDDLE_BLACK)
    right = brightness(right_sensor, RIGHT_WHITE, RIGHT_BLACK)

    if left > 80 and middle > 80 and right > 80:
        # Line lost: keep turning toward the side where it was seen last.
        turn = SEARCH_TURN * line_side
    else:
        # Proportional steering: line under the right sensor -> error > 0 -> turn right.
        error = left - right
        turn = GAIN * error
        if error > 20:
            line_side = 1
        elif error < -20:
            line_side = -1

    # Speed up one wheel and slow down the other to steer.
    left_motor.dc(SPEED + turn)
    right_motor.dc(SPEED - turn)
    wait(10)
