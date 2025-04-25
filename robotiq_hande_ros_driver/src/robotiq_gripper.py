#!/usr/bin/env python3

"""Module to control Robotiq's grippers - tested with HAND-E"""
import socket
import threading
import time
from enum import Enum
from collections import OrderedDict

class RobotiqGripper:
    """
    Communicates with the gripper directly, via socket commands.
    """
    # WRITE VARIABLES
    ACT = 'ACT'  # activate (1 while activated)
    GTO = 'GTO'  # go to
    ATR = 'ATR'  # auto-release
    ADR = 'ADR'  # auto-release direction
    FOR = 'FOR'  # force (0-255)
    SPE = 'SPE'  # speed (0-255)
    POS = 'POS'  # position (0-255)

    # READ VARIABLES
    STA = 'STA'  # status
    PRE = 'PRE'  # position echo
    OBJ = 'OBJ'  # object detection
    FLT = 'FLT'  # fault

    ENCODING = 'UTF-8'

    class GripperStatus(Enum):
        RESET = 0
        ACTIVATING = 1
        ACTIVE = 3

    class ObjectStatus(Enum):
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def __init__(self):
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    def connect(self, hostname, port, socket_timeout=2.0):
        """Connects to the gripper at the given address."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self):
        """Closes the connection."""
        if self.socket:
            self.socket.close()

    def _set_vars(self, var_dict):
        """Sets multiple variables and waits for an ack."""
        cmd = 'SET'
        for variable, value in var_dict.items():
            cmd += f' {variable} {value}'
        cmd += '\n'
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable, value):
        """Sets a single variable."""
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable):
        """Gets a variable's value."""
        with self.command_lock:
            self.socket.sendall(f"GET {variable}\n".encode(self.ENCODING))
            data = self.socket.recv(1024)
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f"Unexpected response: {data}")
        return int(value_str)

    @staticmethod
    def _is_ack(data):
        """Checks for ack response."""
        return data.strip() == b'ack'

    def _reset(self):
        """Resets the gripper (clears faults)."""
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
        time.sleep(0.5)

    def activate(self, auto_calibrate=True):
        """Activates the gripper and optionally auto-calibrates."""
        if not self.is_active():
            self._reset()
            while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
                time.sleep(0.01)
            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while self._get_var(self.ACT) != 1 or self._get_var(self.STA) != 3:
                time.sleep(0.01)
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        """Returns whether the gripper is active."""
        return RobotiqGripper.GripperStatus(self._get_var(self.STA)) == RobotiqGripper.GripperStatus.ACTIVE

    def get_min_position(self):
        """Returns minimum (open) position."""
        return self._min_position

    def get_max_position(self):
        """Returns maximum (closed) position."""
        return self._max_position

    def get_open_position(self):
        """Alias for minimum position."""
        return self.get_min_position()

    def get_closed_position(self):
        """Alias for maximum position."""
        return self.get_max_position()

    def is_open(self):
        """Checks if fully open."""
        return self.get_current_position() <= self.get_open_position()

    def is_closed(self):
        """Checks if fully closed."""
        return self.get_current_position() >= self.get_closed_position()

    def get_current_position(self):
        """Gets current position."""
        return self._get_var(self.POS)

    def auto_calibrate(self, log=True):
        """Auto-calibrates open/closed positions."""
        pos, stat = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(stat) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening: {stat}")
        pos, stat = self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(stat) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed closing: {stat}")
        self._max_position = pos
        pos, stat = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(stat) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed reopening: {stat}")
        self._min_position = pos
        log_fn = getattr(__import__('rospy', fromlist=['loginfo']), 'loginfo', print)
        log_fn(f"Gripper auto-calibrated to [{self._min_position}, {self._max_position}]")

    def move(self, position, speed, force):
        """Non-blocking move command."""
        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))
        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)
        var_dict = OrderedDict([
            (self.POS, clip_pos),
            (self.SPE, clip_spe),
            (self.FOR, clip_for),
            (self.GTO, 1)
        ])
        success = self._set_vars(var_dict)
        return success, clip_pos

    def move_and_wait_for_pos(self, position, speed, force):
        """Blocking move command until completion."""
        success, cmd_pos = self.move(position, speed, force)
        if not success:
            raise RuntimeError('Failed to send move command')
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)
        status = self._get_var(self.OBJ)
        while RobotiqGripper.ObjectStatus(status) == RobotiqGripper.ObjectStatus.MOVING:
            status = self._get_var(self.OBJ)
        final_pos = self._get_var(self.POS)
        return final_pos, status