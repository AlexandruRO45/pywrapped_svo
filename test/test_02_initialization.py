# test/test_02_initialization.py
import pywrapped_svo as svo

def create_valid_config():
    config = svo.Config()
    config.camera_model = "Pinhole"
    config.img_width = 640; config.img_height = 480
    config.fx = 458.654; config.fy = 457.296
    config.cx = 367.215; config.cy = 248.375
    config.k1 = -0.28340811; config.k2 = 0.07395907
    config.p1 = 0.00019359; config.p2 = 1.76187114e-05
    return config

def test_system_initialization_succeeds():
    """
    Tests if a single System instance can be created and successfully initialized.
    """
    print("--- Running Test: Successful Initialization ---")
    system = svo.System()
    config = create_valid_config()
    
    initialization_result = system.initialize(config)
    assert initialization_result is True
    
    # Per our last finding, the state is correctly PAUSED after init.
    # It only changes after the first image is processed.
    assert system.get_tracking_state() == svo.TrackingState.PAUSED
    print("--- PASSED: Successful Initialization ---")
    system = svo.reset()