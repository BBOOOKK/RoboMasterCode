"""
RoboMaster S1 Blue Line Follower - Professional Refactored Version

This module provides a professional implementation of a blue line following algorithm
for the RoboMaster S1 robot. The functionality is identical to the original base1.py
but with improved code structure, organization, and maintainability.

Features:
- Enhanced PID controller for precise line following
- Multiple ROI detection for stability
- Curvature-based speed adjustment
- Line loss recovery with search algorithm
- Professional code organization with proper encapsulation
"""

import cv2
import numpy as np
import time
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass

from robomaster import robot
from robomaster import camera


@dataclass
class LineFollowerConfig:
    """Configuration parameters for the line follower."""
    # Connection settings
    connection_type: str = "ap"
    
    # Color detection settings
    lower_blue: Tuple[int, int, int] = (100, 80, 50)
    upper_blue: Tuple[int, int, int] = (130, 255, 255)
    
    # ROI settings
    roi_row_offsets: Tuple[int, int, int] = (100, 70, 40)
    min_white_pixels: int = 10
    
    # PID settings
    main_pid_params: Tuple[float, float, float, float] = (0.2, 0.001, 0.05, 80)
    curved_pid_params: Tuple[float, float, float, float] = (0.5, 0.002, 0.08, 100)
    
    # Gimbal settings
    gimbal_pitch: float = -25.0
    gimbal_yaw: float = 0.0
    gimbal_speed: int = 100
    
    # Line loss settings
    max_line_lost_frames: int = 10
    search_timeout_seconds: float = 8.0
    search_speed_degrees: float = 20.0
    
    # History settings
    position_history_length: int = 8
    curvature_history_length: int = 5
    
    # Speed settings
    base_speed_straight: float = 0.82
    base_speed_curve: float = 0.58
    
    # Curvature thresholds
    curvature_enter_threshold: float = 0.3
    curvature_exit_threshold: float = 0.2


class EnhancedPID:
    """Enhanced PID controller with integral limiting and output clamping."""
    
    def __init__(self, p: float = 0.75, i: float = 0.09, d: float = 0.05, 
                 out_limit: float = 80, integral_limit: float = 50):
        """
        Initialize the PID controller.
        
        Args:
            p: Proportional gain
            i: Integral gain
            d: Derivative gain
            out_limit: Output limit (clamping)
            integral_limit: Integral term limit (anti-windup)
        """
        self.kp = p
        self.ki = i
        self.kd = d
        self.out_limit = float(out_limit)
        self.integral_limit = float(integral_limit)
        self.clear()
    
    def clear(self) -> None:
        """Reset the PID controller state."""
        self.set_point = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, target: float, current: float) -> float:
        """
        Update the PID controller and return the control output.
        
        Args:
            target: Target value
            current: Current measured value
            
        Returns:
            Control output
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Avoid division by zero
        if dt <= 0:
            dt = 0.01
        
        error = target - current
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with clamping
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error
        
        # Total output with clamping
        output = p_term + i_term + d_term
        output = max(-self.out_limit, min(output, self.out_limit))
        
        return output


class LineFollower:
    """Main line following class with professional structure."""
    
    def __init__(self, config: Optional[LineFollowerConfig] = None):
        """
        Initialize the line follower.
        
        Args:
            config: Configuration parameters (uses defaults if None)
        """
        self.config = config or LineFollowerConfig()
        self._initialize_components()
        self._initialize_state()
    
    def _initialize_components(self) -> None:
        """Initialize robot components and connections."""
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type=self.config.connection_type)
        
        self.ep_camera = self.ep_robot.camera
        self.ep_camera.start_video_stream(display=False)
        
        self.ep_chassis = self.ep_robot.chassis
        self.ep_gimbal = self.ep_robot.gimbal
        
        # Initialize gimbal to fixed position
        self.ep_gimbal.recenter().wait_for_completed()
        self.ep_gimbal.moveto(
            pitch=self.config.gimbal_pitch,
            yaw=self.config.gimbal_yaw,
            pitch_speed=self.config.gimbal_speed,
            yaw_speed=self.config.gimbal_speed
        ).wait_for_completed()
        
        self.ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
        
        # Initialize PID controllers
        self.main_pid = EnhancedPID(
            p=self.config.main_pid_params[0],
            i=self.config.main_pid_params[1],
            d=self.config.main_pid_params[2],
            out_limit=self.config.main_pid_params[3]
        )
        
        self.curved_pid = EnhancedPID(
            p=self.config.curved_pid_params[0],
            i=self.config.curved_pid_params[1],
            d=self.config.curved_pid_params[2],
            out_limit=self.config.curved_pid_params[3]
        )
    
    def _initialize_state(self) -> None:
        """Initialize the internal state variables."""
        self.frame_width: Optional[int] = None
        self.frame_center: Optional[int] = None
        
        self.line_lost_count: int = 0
        self.searching: bool = False
        self.search_direction: int = -1
        self.search_start_time: float = 0.0
        
        self.position_history: List[int] = []
        self.curvature_history: List[float] = []
        
        self.avg_curvature: float = 0.0
        self.last_error: float = 0.0
        self.last_z_speed: float = 0.0
        self.last_x_speed: float = 0.0
    
    @staticmethod
    def calculate_curvature(x_centers: List[int], y_rows: List[int], scale: float = 0.5) -> float:
        """
        Calculate the curvature of the detected line.
        
        Args:
            x_centers: List of x-coordinates of line centers
            y_rows: List of corresponding y-coordinates
            scale: Scaling factor for curvature
            
        Returns:
            Calculated curvature value
        """
        if not x_centers or not y_rows:
            return 0.0
        
        if len(x_centers) != len(y_rows):
            n = min(len(x_centers), len(y_rows))
            x_centers, y_rows = x_centers[:n], y_rows[:n]
        
        if len(x_centers) < 2:
            return 0.0
        
        x = np.asarray(x_centers, dtype=np.float32)
        y = np.asarray(y_rows, dtype=np.float32)
        
        if np.unique(y).size < 2 or np.isnan(x).any() or np.isnan(y).any():
            return 0.0
        
        # Fit line: x = m*y + b
        m, _ = np.polyfit(y, x, 1)
        return float(m) * scale
    
    def _process_frame(self, frame: np.ndarray) -> Tuple[Optional[List[int]], Optional[List[int]]]:
        """
        Process a single frame to detect blue lines.
        
        Args:
            frame: Input frame from camera
            
        Returns:
            Tuple of (line_centers, line_rows) or (None, None) if no lines detected
        """
        if self.frame_width is None:
            height, self.frame_width = frame.shape[:2]
            self.frame_center = self.frame_width // 2
            print(f"Detected image dimensions: width={self.frame_width}, height={height}")
            print(f"Image center: {self.frame_center}")
        
        # Convert to HSV and create blue mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.config.lower_blue), np.array(self.config.upper_blue))
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        # Detect lines in multiple ROI rows
        height, width = mask.shape
        roi_rows = [height - offset for offset in self.config.roi_row_offsets]
        
        line_centers = []
        line_rows = []
        
        for roi_row in roi_rows:
            roi_row = max(0, min(height - 1, roi_row))
            row_pixels = mask[roi_row, :]
            white_pixels = np.where(row_pixels == 255)[0]
            
            if len(white_pixels) > self.config.min_white_pixels:
                center_x = int(np.mean(white_pixels))
                line_centers.append(center_x)
                line_rows.append(roi_row)
                
                # Draw debug information
                cv2.line(frame, (0, roi_row), (width, roi_row), (0, 255, 0), 1)
                cv2.circle(frame, (center_x, roi_row), 5, (0, 0, 255), -1)
        
        return (line_centers, line_rows) if line_centers else (None, None)
    
    def _handle_line_detected(self, line_centers: List[int], line_rows: List[int], 
                             frame: np.ndarray) -> None:
        """
        Handle the case when lines are detected.
        
        Args:
            line_centers: Detected line center positions
            line_rows: Corresponding row positions
            frame: Current frame for debugging
        """
        if self.searching:
            self.searching = False
            print("Line found, returning to tracking mode")
            self.main_pid.clear()
        
        # Calculate smoothed center position
        avg_center = int(np.mean(line_centers))
        self.position_history.append(avg_center)
        if len(self.position_history) > self.config.position_history_length:
            self.position_history.pop(0)
        
        smoothed_center = int(np.mean(self.position_history))
        
        # Calculate curvature
        curvature = self.calculate_curvature(line_centers, line_rows)
        self.curvature_history.append(curvature)
        if len(self.curvature_history) > self.config.curvature_history_length:
            self.curvature_history.pop(0)
        
        self.avg_curvature = np.mean(self.curvature_history)
        
        # Calculate error
        error = self.frame_center - smoothed_center
        
        # Select appropriate PID controller based on curvature
        use_curve_pid = abs(self.avg_curvature) > self.config.curvature_enter_threshold
        
        if use_curve_pid:
            z_speed = self.curved_pid.update(0, error)
            x_speed = self.config.base_speed_curve
        else:
            z_speed = self.main_pid.update(0, error)
            x_speed = self.config.base_speed_straight
        
        # Send control commands
        self.ep_chassis.drive_speed(x=x_speed, y=0, z=z_speed, timeout=0.1)
        
        # Update state
        self.last_error = float(error)
        self.last_z_speed = float(z_speed)
        self.last_x_speed = float(x_speed)
        self.line_lost_count = 0
        
        # Draw debug information
        height, _ = frame.shape[:2]
        cv2.circle(frame, (smoothed_center, height - 50), 8, (255, 0, 0), -1)
        cv2.circle(frame, (self.frame_center, height - 50), 8, (0, 255, 255), -1)
        
        cv2.putText(frame, f"Error: {error}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Z-Speed: {z_speed:.1f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Curvature: {self.avg_curvature:.2f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    def _handle_line_lost(self, frame: np.ndarray) -> None:
        """
        Handle the case when no lines are detected.
        
        Args:
            frame: Current frame for debugging
        """
        self.line_lost_count += 1
        
        if self.line_lost_count > self.config.max_line_lost_frames:
            if not self.searching:
                self.searching = True
                self.search_start_time = time.time()
                print("Line lost, starting search...")
            
            elapsed_time = time.time() - self.search_start_time
            height, width = frame.shape[:2]
            
            if elapsed_time > self.config.search_timeout_seconds:
                # Timeout - change direction
                self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                self.search_direction *= -1
                self.search_start_time = time.time()
                
                cv2.putText(frame, "SEARCH TIMEOUT - CHANGING DIR",
                           (width // 2 - 150, height // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # Continue searching
                search_speed = self.config.search_speed_degrees * self.search_direction
                self.ep_chassis.drive_speed(x=0, y=0, z=search_speed, timeout=0.1)
                self.last_z_speed = float(search_speed)
                self.last_x_speed = 0.0
                
                cv2.putText(frame, f"SEARCHING... {int(self.config.search_timeout_seconds - elapsed_time)}s",
                           (width // 2 - 100, height // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    def run(self) -> None:
        """Main execution loop for the line follower."""
        try:
            while True:
                frame = self.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                if frame is None:
                    continue
                
                line_centers, line_rows = self._process_frame(frame)
                
                if line_centers:
                    self._handle_line_detected(line_centers, line_rows, frame)
                else:
                    self._handle_line_lost(frame)
                
                # Display debug windows
                cv2.imshow("Frame", frame)
                cv2.imshow("Mask", cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources and stop the robot."""
        try:
            self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(0.5)
            self.ep_camera.stop_video_stream()
            self.ep_robot.close()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Error during cleanup: {e}")


def main():
    """Main entry point for the line follower application."""
    print("Starting RoboMaster S1 Line Follower...")
    print("Press 'q' to quit")
    
    follower = LineFollower()
    follower.run()


if __name__ == "__main__":
    main()
