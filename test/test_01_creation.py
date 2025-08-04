# test/test_01_creation.py
import pywrapped_svo as svo

def test_system_object_can_be_created():
    """
    Tests the most basic step: creating the System object.
    This does not call initialize() and should not trigger the SVO engine.
    """
    print("--- Running Test: Object Creation ---")
    system = svo.System()
    assert system is not None
    assert system.get_tracking_state() == svo.TrackingState.PAUSED
    print("--- PASSED: Object Creation ---")
    system = svo.reset()