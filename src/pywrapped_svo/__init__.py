"""
rpg_svo Python bindings

This package provides Python bindings for the Semi-Direct Visual Odometry system.
"""


from ._version import __version__, __url__#, __dependencies__

try:
    # From the file `_core.so`, import the bound C++ classes and enums.
    from ._core import Config, ProcessResult, System
    from ._core import TrackingState, TrackingQuality
    from ._core import NodeData, EdgeData, PointData

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
    "TrackingState",        #
    "TrackingQuality",      #
    "Config",               #
    "NodeData",             #
    "EdgeData",             #
    "PointData",            # 3D MapPoint data
    "ProcessResult",        # Main processing function, takes an image and returns the latest pose and state
    "System"                # The main class for interacting with Semi-direct Visual Odometry
]