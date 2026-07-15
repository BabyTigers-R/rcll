"""Named waypoints for myAGV navigation, expressed in the `map` frame
(the same frame shown in RViz2 once AMCL is running).
"""
from math import radians

# name -> (x [m], y [m], yaw [deg])
LOCATIONS = {
    'start': (0.0, 0.0, 0.0),
    'goal': (2.0, 1.0, 90.0),
    'mps_1': (1.0, 0.5, 0.0),
    'mps_2': (1.0, -0.5, 0.0),
}


def list_location_names():
    """Return all registered location names, sorted."""
    return sorted(LOCATIONS.keys())


def get_location(name):
    """Return (x, y, yaw_rad) for a named location.

    Raises KeyError if `name` is not registered in LOCATIONS.
    """
    x, y, yaw_deg = LOCATIONS[name]
    return x, y, radians(yaw_deg)
