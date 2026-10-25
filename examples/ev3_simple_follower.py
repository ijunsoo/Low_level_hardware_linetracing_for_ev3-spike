# Simple line follower for a LEGO MINDSTORMS EV3 brick running Pybricks.
#
# Robot: two motors per side (A + B left, C + D right) and three EV3 Color
# Sensors in a row in front of the wheels: S1 = left, S2 = middle, S3 = right.
#
# Simulator (from the repository root):
#   python sim/run_sim.py examples/ev3_simple_follower.py --profile ev3_3s_geo --track main --no-plot

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# The right motors are mounted mirrored, so they turn counterclockwise to drive forward.
left_motors = [Motor(Port.A), Motor(Port.B)]
right_motors = [Motor(Port.C, Direction.COUNTERCLOCKWISE),
                Motor(Port.D, Direction.COUNTERCLOCKWISE)]

left_sensor = ColorSensor(Port.S1)
middle_sensor = ColorSensor(Port.S2)
right_sensor = ColorSensor(Port.S3)

# Reflection each sensor reads on the white floor and on the black line.
# Measure your own robot (print(sensor.reflection())) and put the numbers here.
LEFT_WHITE, LEFT_BLACK = 34, 6
MIDDLE_WHITE, MIDDLE_BLACK = 21, 3
RIGHT_WHITE, RIGHT_BLACK = 22, 3

SPEED = 40        # forward power in % (motor duty)
GAIN = 0.4        # steering per point of brightness difference
SEARCH_TURN = 60  # steering while no sensor sees the line


def brightness(sensor, white, black):
    # Scale the reading to 0 (black line) ... 100 (white floor).
    return (sensor.reflection() - black) * 100 / (white - black)


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

    # Speed up one side and slow down the other to steer.
    for motor in left_motors:
        motor.dc(SPEED + turn)
    for motor in right_motors:
        motor.dc(SPEED - turn)
    wait(10)
