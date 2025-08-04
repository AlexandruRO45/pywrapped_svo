import os
import argparse
import cv2
import pywrapped_svo as svo
import numpy as np

def create_svo_config():
    """Configures SVO using the parameters from the sin2_tex2_h1_v8_d dataset."""
    config = svo.Config()
    config.camera_model = "Pinhole"
    config.img_width = 752
    config.img_height = 480
    config.fx = 315.5; config.fy = 315.5
    config.cx = 376.0; config.cy = 240.0
    # config.k1 = 0.0; config.k2 = 0.0; config.p1 = 0.0; config.p2 = 0.0
    return config

def main():
    parser = argparse.ArgumentParser(description="Run the SVO wrapper on a directory of images.")
    parser.add_argument("dataset_dir", help="Path to the root directory of the dataset (e.g., sin2_tex2_h1_v8_d).")
    args = parser.parse_args()

    img_dir = os.path.join(args.dataset_dir, 'img')
    if not os.path.isdir(img_dir):
        print(f"Error: 'img' directory not found in '{args.dataset_dir}'")
        return

    print("Initializing SVO System...")
    svo.System.configure(create_svo_config())
    system = svo.System()
    if not system.initialize(): # Pass config again for camera geometry
        print("Error: SVO System initialization failed!")
        return
    print("System instance created and initialized successfully.")
    print("System initialized successfully.")

    try:
        # Replicate the C++ benchmark loop from frame 2 to 187
        for img_id in range(2, 188):
            filename = f"frame_{img_id:06d}_0.png"
            image_path = os.path.join(img_dir, filename)
            image = cv2.imread(image_path, 0)
            if image is None:
                print(f"Warning: Could not read image {image_path}, stopping.")
                break
            
            # image = np.ascontiguousarray(image)
            timestamp = float(img_id) * 0.01
            result = system.process_image(image, timestamp)
            
            # Use the newly exposed functions for detailed output
            frame_id = system.get_last_frame_id()
            num_obs = system.get_last_num_observations()
            proc_time = system.get_last_processing_time() * 1000  # ms

            if frame_id != -1:
                print(
                    f"Frame-Id: {frame_id} | "
                    f"#Features: {num_obs} | "
                    f"Time: {proc_time:.3f}ms | "
                    f"State: {result.stage.name}"
                )

    except Exception as e:
        print(f"\nAn error occurred during processing: {e}")
    finally:
        print("\nProcessing finished.")

if __name__ == "__main__":
    main()