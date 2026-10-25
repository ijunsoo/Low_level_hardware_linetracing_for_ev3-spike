# MicroPython의 micropython 모듈 대신 (시뮬레이터 = CPython): 실기 코드가 import해도 돌게
def const(x):
    return x


def native(f):
    return f


viper = native


def mem_info(*a):
    pass


def opt_level(*a):
    return 0


def kbd_intr(*a):
    pass


def schedule(f, arg):
    f(arg)
