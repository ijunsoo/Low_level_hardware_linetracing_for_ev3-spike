# Line follower in the style of the official Pybricks docs: SPIKE Color
# Sensors and a DriveBase, for a LEGO SPIKE Prime hub.
#
# Robot: left motor on port E, right motor on port F, and three Color Sensors
# in a row in front of the wheels: C = left, A = middle, B = right.
#
# Simulator (from the repository root):
#   python sim/run_sim.py examples/spike_drivebase_follower.py --profile spike_3s_geo2 --track main --no-plot

from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# The right motor is mounted mirrored, so it turns counterclockwise to drive forward.
left_motor = Motor(Port.E)
right_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, wheel_diameter=80, axle_track=205)

left_sensor = ColorSensor(Port.C)
middle_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.B)

# Reflection each sensor reads on the white floor and on the black line.
# Measure your own robot (print(sensor.reflection())) and put the numbers here.
LEFT_WHITE, LEFT_BLACK = 34, 6
MIDDLE_WHITE, MIDDLE_BLACK = 21, 3
RIGHT_WHITE, RIGHT_BLACK = 22, 3

SPEED = 250        # forward speed in mm/s
GAIN = 2           # turn rate (deg/s) per point of brightness difference
SEARCH_RATE = 250  # turn rate (deg/s) while no sensor sees the line


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
        robot.drive(SPEED / 2, SEARCH_RATE * line_side)
    else:
        # Line under the right sensor -> error > 0 -> turn right (clockwise is positive).
        error = left - right
        if error > 20:
            line_side = 1
        elif error < -20:
            line_side = -1
        robot.drive(SPEED, GAIN * error)
    wait(10)
