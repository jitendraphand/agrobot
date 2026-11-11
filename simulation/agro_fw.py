"""ctypes binding to the firmware control core (firmware/lib/agro_core).

The simulator does not re-implement any control logic: it loads the same C++
that is flashed onto the ESP32 and feeds it simulated sensor readings.
"""
import ctypes
import os
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOST = os.path.join(ROOT, "firmware", "test_host")
LIB = os.path.join(HOST, "build", "libagrocore.so")


class AgroIn(ctypes.Structure):
    _fields_ = [("dt", ctypes.c_float), ("encL", ctypes.c_int), ("encR", ctypes.c_int),
                ("imuValid", ctypes.c_int), ("gyroZ", ctypes.c_float), ("roll", ctypes.c_float),
                ("pitch", ctypes.c_float), ("battV", ctypes.c_float), ("soilRaw", ctypes.c_int),
                ("rangeM", ctypes.c_float), ("estop", ctypes.c_int)]


class AgroOut(ctypes.Structure):
    _fields_ = [("dutyL", ctypes.c_float), ("dutyR", ctypes.c_float), ("motorsEnabled", ctypes.c_int),
                ("joint", ctypes.c_float * 5), ("servosEnabled", ctypes.c_int), ("pump", ctypes.c_int),
                ("led", ctypes.c_int), ("state", ctypes.c_int), ("busy", ctypes.c_int),
                ("odomX", ctypes.c_float), ("odomY", ctypes.c_float), ("odomTh", ctypes.c_float)]


STATES = ["IDLE", "TELEOP", "MISSION", "ESTOP", "FAULT"]


def build():
    subprocess.run(["make", "-s", "-C", HOST, "lib"], check=True)


class Firmware:
    """One instance of the robot controller."""

    def __init__(self):
        build()
        self.lib = ctypes.CDLL(LIB)
        self.lib.agro_create.restype = ctypes.c_void_p
        self.lib.agro_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(AgroIn), ctypes.POINTER(AgroOut)]
        self.lib.agro_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        self.lib.agro_command.restype = ctypes.c_int
        self.lib.agro_destroy.argtypes = [ctypes.c_void_p]
        self.lib.agro_ik.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.agro_fk.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.agro_grip_angle.argtypes = [ctypes.c_float]
        self.lib.agro_grip_angle.restype = ctypes.c_float
        self.h = ctypes.c_void_p(self.lib.agro_create())
        self.inp = AgroIn()
        self.out = AgroOut()
        self._reply = ctypes.create_string_buffer(1024)

    def step(self):
        self.lib.agro_step(self.h, ctypes.byref(self.inp), ctypes.byref(self.out))
        return self.out

    def command(self, line):
        ok = self.lib.agro_command(self.h, line.encode(), self._reply, len(self._reply))
        return bool(ok), self._reply.value.decode()

    def ik(self, x, y, z, pitch, tool=0):
        q = (ctypes.c_float * 4)()
        r = self.lib.agro_ik(x, y, z, pitch, tool, q)
        return r, list(q)

    def fk(self, q, tool=0):
        qa = (ctypes.c_float * 4)(*q)
        out = (ctypes.c_float * 4)()
        self.lib.agro_fk(qa, tool, out)
        return list(out)

    @property
    def state(self):
        return STATES[self.out.state]
