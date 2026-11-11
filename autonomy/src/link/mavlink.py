"""The link to PX4 over MAVLink 2, with the same step(state) as link/luckyLink.py. It talks to PX4's API link (UDP 14580
in SITL), the way a GCS does: a HEARTBEAT each second (PX4 answers whoever sends first), the sticks as MANUAL_CONTROL,
each new command once, and the setpoint as SET_POSITION_TARGET_LOCAL_NED in OFFBOARD. state.py keeps ArduPilot's mode
numbers: they are turned into PX4's modes here, and back.

  rc          -> MANUAL_CONTROL (x pitch forward, y roll right, z throttle 0..1000 with 500 holding, r yaw right)
  "arm"       -> MAV_CMD_COMPONENT_ARM_DISARM
  "mode"      -> MAV_CMD_DO_SET_MODE: LOITER POSCTL, ALT_HOLD ALTCTL, RTL AUTO.RTL, GUIDED OFFBOARD
  "takeoff"   -> MAV_CMD_NAV_TAKEOFF, value metres above home
  "gripper"   -> MAV_CMD_DO_GRIPPER (1 grab, 0 release)
  setpoint    -> SET_POSITION_TARGET_LOCAL_NED, and OFFBOARD while it is set
  armed, mode <- HEARTBEAT
  pos, vel    <- LOCAL_POSITION_NED
  att         <- ATTITUDE
  landed      <- EXTENDED_SYS_STATE
  gripper     <- the last gripper command PX4 took
  ack         <- COMMAND_ACK
  connected   <- a HEARTBEAT in the last second
"""
import math
import os
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

from state import ALT_HOLD, GUIDED, LOITER, RTL

URL = "udpout:127.0.0.1:14580"  # PX4 SITL's API (offboard) link
TIMEOUT = 1.0  # s without PX4's HEARTBEAT: not connected
m = mavutil.mavlink


def px4(main, sub=0):
    """PX4's custom_mode for a main mode and sub mode."""
    return main << 16 | sub << 24


POSCTL, ALTCTL, OFFBOARD = px4(3), px4(2), px4(6)
TAKEOFF, HOLD, RETURN, LAND = px4(4, 2), px4(4, 3), px4(4, 5), px4(4, 6)
TO_PX4 = {LOITER: POSCTL, ALT_HOLD: ALTCTL, RTL: RETURN, GUIDED: OFFBOARD}
FROM_PX4 = {POSCTL: LOITER, ALTCTL: ALT_HOLD, OFFBOARD: GUIDED, TAKEOFF: GUIDED, HOLD: GUIDED, RETURN: RTL, LAND: RTL}
COMMANDS = {"arm": m.MAV_CMD_COMPONENT_ARM_DISARM, "mode": m.MAV_CMD_DO_SET_MODE, "takeoff": m.MAV_CMD_NAV_TAKEOFF,
            "gripper": m.MAV_CMD_DO_GRIPPER}
POSITION_ONLY = 0x9F8  # SET_POSITION_TARGET's type_mask: position and yaw, no velocity, acceleration or yaw rate


class MavLink:
    def __init__(self, url=URL):
        self.mav = mavutil.mavlink_connection(url, source_system=255, source_component=m.MAV_COMP_ID_MISSIONPLANNER)
        self.last = -TIMEOUT  # PX4's last HEARTBEAT
        self.beat = -TIMEOUT  # our last HEARTBEAT
        self.offboard = -TIMEOUT  # our last OFFBOARD request
        self.sent = None  # the seq of the last command sent
        self.home = 0.0  # m, home's altitude above sea level
        self.px4_mode = None

    def command(self, cmd, *params):
        p = (list(params) + [0.0] * 7)[:7]
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, cmd, 0, *p)

    def set_mode(self, custom_mode):
        self.command(m.MAV_CMD_DO_SET_MODE, m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode >> 16 & 0xFF,
                     custom_mode >> 24 & 0xFF)

    def receive(self, state):
        while (msg := self.mav.recv_match(blocking=False)) is not None:
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.autopilot == m.MAV_AUTOPILOT_PX4:
                self.mav.target_system, self.mav.target_component = msg.get_srcSystem(), msg.get_srcComponent()
                state.armed = bool(msg.base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED)
                self.px4_mode = msg.custom_mode
                state.mode = FROM_PX4.get(msg.custom_mode, msg.custom_mode)  # another PX4 mode shows as its number
                self.last = time.monotonic()
            elif t == "LOCAL_POSITION_NED":
                state.pos, state.vel = (msg.x, msg.y, msg.z), (msg.vx, msg.vy, msg.vz)
            elif t == "ATTITUDE":
                state.att = (msg.roll, msg.pitch, msg.yaw)
            elif t == "EXTENDED_SYS_STATE":
                state.landed = msg.landed_state == m.MAV_LANDED_STATE_ON_GROUND
            elif t == "GLOBAL_POSITION_INT":
                self.home = (msg.alt - msg.relative_alt) / 1000
            elif t == "COMMAND_ACK" and self.sent is not None and msg.command == COMMANDS.get(state.cmd):
                # SITL has no gripper: PX4 answers UNSUPPORTED, so the gripper is taken as moved anyway.
                gripper = state.cmd == "gripper" and msg.result == m.MAV_RESULT_UNSUPPORTED
                if msg.result == m.MAV_RESULT_ACCEPTED or gripper:
                    state.ack = self.sent
                    if state.cmd == "gripper":
                        state.gripper = bool(state.value)

    def step(self, state):
        now = time.monotonic()
        if now - self.beat >= 1.0:
            self.mav.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, m.MAV_STATE_ACTIVE)
            self.beat = now
        self.receive(state)
        state.connected = now - self.last < TIMEOUT
        if not state.connected:
            return
        roll, pitch, throttle, yaw = state.rc
        self.mav.mav.manual_control_send(self.mav.target_system, 2 * (1500 - pitch), 2 * (roll - 1500),
                                         throttle - 1000, 2 * (yaw - 1500), 0)
        if state.seq != self.sent and state.cmd in COMMANDS:
            self.sent = state.seq
            if state.cmd == "mode":
                self.set_mode(TO_PX4[int(state.value)])
            elif state.cmd == "takeoff":
                self.command(m.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, self.home + state.value)
            elif state.cmd == "gripper":
                self.command(m.MAV_CMD_DO_GRIPPER, 1, m.GRIPPER_ACTION_GRAB if state.value else m.GRIPPER_ACTION_RELEASE)
            else:
                self.command(COMMANDS[state.cmd], state.value)
        if state.setpoint is not None:
            n, e, d, yaw = state.setpoint
            self.mav.mav.set_position_target_local_ned_send(0, self.mav.target_system, self.mav.target_component,
                                                            m.MAV_FRAME_LOCAL_NED, POSITION_ONLY, n, e, d,
                                                            0, 0, 0, 0, 0, 0, yaw, 0)
            if self.px4_mode != OFFBOARD and now - self.offboard >= 1.0:  # PX4 takes OFFBOARD once setpoints arrive
                self.set_mode(OFFBOARD)
                self.offboard = now
