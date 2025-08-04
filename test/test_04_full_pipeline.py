# test/test_04_full_pipeline.py
import numpy as np
import pywrapped_svo as svo
import cv2

def create_valid_config():
    config = svo.Config()
    config.camera_model = "Pinhole"
    config.img_width = 640; config.img_height = 480
    config.fx = 458.654; config.fy = 457.296
    config.cx = 367.215; config.cy = 248.375
    config.k1 = -0.28340811; config.k2 = 0.07395907
    config.p1 = 0.00019359; config.p2 = 1.76187114e-05
    return config

def create_test_image(width=640, height=480):
    """
    Creates a deterministic image with a grid of white dots.
    This is guaranteed to provide stable features for the SVO FAST detector.
    """
    image = np.zeros((height, width), dtype=np.uint8)
    spacing = 40
    # Draw a grid of circles
    for y in range(spacing, height, spacing):
        for x in range(spacing, width, spacing):
            cv2.circle(image, (x, y), 5, (255), -1)
    return image

def test_full_pipeline_on_single_instance():
    """
    Tests the entire pipeline from init to reset on a single object.
    """
    print("--- Running Test: Full Pipeline ---")
    system = svo.System()
    system.initialize(create_valid_config())
    print("Initialized.")

    # Process first frame
    img1 = create_test_image()
    system.process_image(img1, 0.0)
    assert system.get_tracking_state() == svo.TrackingState.SECOND_FRAME
    print("Processed Frame 1.")

    # Process second frame
    img2 = np.roll(img1, 20, axis=1)
    system.process_image(img2, 0.1)
    assert system.get_tracking_state() == svo.TrackingState.DEFAULT_FRAME
    print("Processed Frame 2.")

    # Check map
    nodes = system.get_all_nodes()
    assert len(nodes) == 2
    print("Map data verified.")

    # Reset
    system.reset()
    assert system.get_tracking_state() == svo.TrackingState.PAUSED
    assert len(system.get_all_nodes()) == 0
    print("System reset successfully.")
    print("--- PASSED: Full Pipeline ---")