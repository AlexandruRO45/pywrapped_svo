# test/test_03_init_failure.py
import pywrapped_svo as svo

def create_invalid_config():
    config = svo.Config()
    config.camera_model = "this_model_does_not_exist"
    return config

def test_system_initialization_fails_gracefully():
    """
    Tests if initialization fails correctly with a bad configuration.
    """
    print("--- Running Test: Graceful Init Failure ---")
    system = svo.System()
    config = create_invalid_config()
    assert system.initialize(config) is False
    print("--- PASSED: Graceful Init Failure ---")
    system = svo.reset()