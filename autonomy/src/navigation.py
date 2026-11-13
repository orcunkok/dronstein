"""Navigation: position setpoints for the FC (GUIDED; PX4's OFFBOARD). navigate() sets the point the drone should
fly to (north, east, down) and its heading; the FC flies there and holds it."""

def navigate(state, n, e, d, yaw):
    """Sets the point to fly to: north, east, down in metres, and the heading yaw in radians (from north, clockwise)."""
    state.setpoint = [n, e, d, yaw]

