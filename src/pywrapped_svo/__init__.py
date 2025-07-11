"""
rpg_svo Python bindings

This package provides Python bindings for the Semi-Direct Visual Odometry system.
"""


from ._version import __version__, __url__#, __dependencies__

try:
    # TODO
    # From the file `_core.so`, import the bound C++ classes and enums.
    print("TODO")
    # from ._core import 
    # from ._core import 

except ImportError as e:
    # This provides a much better error message if the C++ part failed.
    # Include the original error for more detailed debugging.
    raise ImportError(
        "Failed to import the compiled svo C++ core (_core.so).\n"
        "Please make sure the package was installed correctly after a full compilation.\n"
        f"Original error: {e}"
    ) from e



# ---- APIs -----
__all__ = [
    "__version__",          # The current version of the compiled ORB-SLAM3 c++ core library file
    "__url__",              # The Github page where the project is located
    "__dependencies__",     # The pip packages must have installed 
    # TODO
    # "System",               # The main class for interacting with SLAM
    # "IMU",                  # The IMU class for handling inertial measurements
    # "Sensor",               # The sensor enum
    # "TrackingState",        # The tracking state enum
]