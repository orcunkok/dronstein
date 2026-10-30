"""The link to ArduPilot over MAVLink, with the same step(state) as lucky/link.py. Not built yet. Connection: SITL
udpin:0.0.0.0:14550, the real X500 the Pi's /dev/ttyAMA0 to the Pixhawk's TELEM2. The plan, State field by field:

  rc          -> RC_CHANNELS_OVERRIDE, chan1-4
  "arm"       -> COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM, param1 = value
  "mode"      -> MAV_CMD_DO_SET_MODE, custom mode = value
  "takeoff"   -> GUIDED, then MAV_CMD_NAV_TAKEOFF, param7 = value
  "gripper"   -> MAV_CMD_DO_GRIPPER, param2 = value (1 grab, 0 release)
  armed, mode <- HEARTBEAT (armed flag, custom_mode)
  pos, vel    <- LOCAL_POSITION_NED
  att         <- ATTITUDE
  landed      <- EXTENDED_SYS_STATE landed_state
  gripper     <- the last MAV_CMD_DO_GRIPPER the FC accepted
  ack         <- COMMAND_ACK
  connected   <- a HEARTBEAT in the last second
"""


class ArduLink:
    def __init__(self):
        raise SystemExit("ArduLink not implemented yet")

    def step(self, state):
        pass
