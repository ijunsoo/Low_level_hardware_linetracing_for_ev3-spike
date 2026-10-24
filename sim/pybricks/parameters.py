class _Named:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


class Port:
    # SPIKE / EV3 모터 포트
    A = _Named("A"); B = _Named("B"); C = _Named("C")
    D = _Named("D"); E = _Named("E"); F = _Named("F")
    # EV3 센서 포트
    S1 = _Named("S1"); S2 = _Named("S2"); S3 = _Named("S3"); S4 = _Named("S4")


class Direction:
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


class Axis:
    X = _Named("X"); Y = _Named("Y"); Z = _Named("Z")


class Stop:
    COAST = _Named("COAST"); BRAKE = _Named("BRAKE"); HOLD = _Named("HOLD")


class Button:
    LEFT = _Named("LEFT"); RIGHT = _Named("RIGHT")
    CENTER = _Named("CENTER"); BLUETOOTH = _Named("BLUETOOTH")
    UP = _Named("UP"); DOWN = _Named("DOWN")                 # EV3


class Color:
    NONE = _Named("NONE")
    BLACK = _Named("BLACK"); WHITE = _Named("WHITE"); GRAY = _Named("GRAY")
    RED = _Named("RED"); GREEN = _Named("GREEN"); BLUE = _Named("BLUE")
    YELLOW = _Named("YELLOW"); ORANGE = _Named("ORANGE"); BROWN = _Named("BROWN")
    CYAN = _Named("CYAN"); MAGENTA = _Named("MAGENTA"); VIOLET = _Named("VIOLET")
