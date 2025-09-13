"""
Test script for line detection functionality.
This module tests the line detection algorithms independently.
"""

import cv2
import numpy as np
from pathlib import Path

# Add the parent directory to the path so we can import modules
import sys
sys.path.append(str(Path(__file__).parent.parent / '1_code'))

from utils.image_processing import (
    create_color_mask, 
    detect_line_centers, 
    denoise_image,
    calculate_frame_center
)


def test_line_detection_with_sample_image():
    """Test line detection using a sample image or simulated data."""
    print("Testing line detection functionality...")
    
    # Create a test image (simulating a blue line on black background)
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Draw a blue line in the center
    cv2.line(test_image, (320, 400), (320, 300), (255, 0, 0), 10)
    cv2.line(test_image, (330, 400), (340, 300), (255, 0, 0), 8)
    
    print(f"Test image shape: {test_image.shape}")
    
    # Test color mask creation
    lower_blue = (100, 80, 50)
    upper_blue = (130, 255, 255)
    
    mask = create_color_mask(test_image, lower_blue, upper_blue)
    print(f"Mask created successfully. Mask shape: {mask.shape}")
    
    # Test denoising
    denoised_mask = denoise_image(mask)
    print("Image denoising completed.")
    
    # Test line center detection
    roi_rows = [400, 350, 300]  # Test rows
    line_centers, valid_rows = detect_line_centers(denoised_mask, roi_rows, min_pixels=5)
    
    print(f"Detected line centers: {line_centers}")
    print(f"Valid rows: {valid_rows}")
    
    # Test frame center calculation
    frame_center = calculate_frame_center(test_image)
    print(f"Frame center: {frame_center}")
    
    if line_centers:
        print("✓ Line detection test PASSED")
        return True
    else:
        print("✗ Line detection test FAILED")
        return False


def test_curvature_calculation():
    """Test curvature calculation with sample data."""
    print("\nTesting curvature calculation...")
    
    from core.robot_control import LineFollower
    
    # Test data: straight line (should have low curvature)
    straight_x = [100, 100, 100, 100]
    straight_y = [400, 350, 300, 250]
    straight_curvature = LineFollower.calculate_curvature(straight_x, straight_y)
    print(f"Straight line curvature: {straight_curvature:.3f}")
    
    # Test data: curved line (should have higher curvature)
    curved_x = [100, 120, 140, 160]
    curved_y = [400, 350, 300, 250]
    curved_curvature = LineFollower.calculate_curvature(curved_x, curved_y)
    print(f"Curved line curvature: {curved_curvature:.3f}")
    
    if abs(straight_curvature) < 0.1 and abs(curved_curvature) > 0.2:
        print("✓ Curvature calculation test PASSED")
        return True
    else:
        print("✗ Curvature calculation test FAILED")
        return False


if __name__ == "__main__":
    print("Running line detection tests...")
    print("=" * 50)
    
    test1_passed = test_line_detection_with_sample_image()
    test2_passed = test_curvature_calculation()
    
    print("=" * 50)
    if test1_passed and test2_passed:
        print("All tests PASSED! ✅")
    else:
        print("Some tests FAILED! ❌")
