"""
Test script for PID control functionality.
This module tests the PID controller algorithms independently.
"""

import numpy as np
from pathlib import Path

# Add the parent directory to the path so we can import modules
import sys
sys.path.append(str(Path(__file__).parent.parent / '1_code'))

from core.robot_control import EnhancedPID


def test_pid_controller_basic():
    """Test basic PID controller functionality."""
    print("Testing basic PID controller...")
    
    # Create PID controller
    pid = EnhancedPID(p=1.0, i=0.1, d=0.05, out_limit=100)
    
    # Test step response
    target = 0.0
    current_values = [10.0, 8.0, 5.0, 2.0, 0.5, 0.0]  # Simulated convergence
    
    outputs = []
    for current in current_values:
        output = pid.update(target, current)
        outputs.append(output)
        print(f"Current: {current:6.2f}, Output: {output:6.2f}")
    
    # Check if outputs are within limits and show convergence
    all_within_limits = all(-100 <= out <= 100 for out in outputs)
    converging = abs(outputs[-1]) < abs(outputs[0])  # Output should decrease as error decreases
    
    print(f"All outputs within limits: {all_within_limits}")
    print(f"Output converging: {converging}")
    
    if all_within_limits and converging:
        print("✓ Basic PID test PASSED")
        return True
    else:
        print("✗ Basic PID test FAILED")
        return False


def test_pid_integral_windup():
    """Test PID integral windup protection."""
    print("\nTesting PID integral windup protection...")
    
    pid = EnhancedPID(p=1.0, i=0.5, d=0.0, out_limit=50, integral_limit=20)
    
    # Simulate sustained error that would cause windup
    target = 0.0
    sustained_error = 10.0
    
    # Apply same error multiple times
    outputs = []
    for _ in range(10):
        output = pid.update(target, sustained_error)
        outputs.append(output)
    
    print(f"Outputs with sustained error: {outputs}")
    
    # Check that integral term is limited
    integral_growth_limited = max(outputs) <= 50  # Should not exceed output limit
    
    # Reset and test negative error
    pid.clear()
    negative_outputs = []
    for _ in range(10):
        output = pid.update(target, -sustained_error)
        negative_outputs.append(output)
    
    print(f"Outputs with negative sustained error: {negative_outputs}")
    negative_limited = min(negative_outputs) >= -50
    
    if integral_growth_limited and negative_limited:
        print("✓ Integral windup protection test PASSED")
        return True
    else:
        print("✗ Integral windup protection test FAILED")
        return False


def test_pid_controller_reset():
    """Test PID controller reset functionality."""
    print("\nTesting PID controller reset...")
    
    pid = EnhancedPID(p=1.0, i=0.1, d=0.05)
    
    # Accumulate some state
    pid.update(0, 10)
    pid.update(0, 8)
    
    # Store current state
    before_reset_integral = pid.integral
    before_reset_error = pid.last_error
    
    # Reset
    pid.clear()
    
    # Check if reset properly
    reset_correct = (pid.integral == 0.0 and 
                    pid.last_error == 0.0 and 
                    pid.set_point == 0.0)
    
    print(f"Before reset - Integral: {before_reset_integral:.3f}, Error: {before_reset_error:.3f}")
    print(f"After reset - Integral: {pid.integral:.3f}, Error: {pid.last_error:.3f}")
    print(f"Reset correctly: {reset_correct}")
    
    if reset_correct:
        print("✓ PID reset test PASSED")
        return True
    else:
        print("✗ PID reset test FAILED")
        return False


if __name__ == "__main__":
    print("Running PID control tests...")
    print("=" * 50)
    
    test1_passed = test_pid_controller_basic()
    test2_passed = test_pid_integral_windup()
    test3_passed = test_pid_controller_reset()
    
    print("=" * 50)
    if test1_passed and test2_passed and test3_passed:
        print("All PID tests PASSED! ✅")
    else:
        print("Some PID tests FAILED! ❌")
