import sys
import os
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
import keyboard
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import math

from lib.PioneerP3DX import PioneerP3DX
from lib.SickTIM310 import SickTIM310

# Utility Functions
def index_to_angle(idx, n):
    """
    Map the index of the laser data (idx) to an angle in the range [-135°, +135°].
    
    Parameters:
        idx: The index of the current data point.
        n: The total number of laser data points.
    Returns:
        The angle corresponding to this index.
    """
    return (idx / (n - 1)) * 270 - 135


def get_front_region(distances, lower_angle=-60, upper_angle=60):
    """
    Extract data within a specified range from the laser data.

    Parameters:
        distances: List of laser data distances.
        lower_angle: Lower bound angle.
        upper_angle: Upper bound angle.
    Returns:
        A list of laser data points within the specified region.
    """
    n = len(distances)
    lower_index = int(((lower_angle + 135) / 270) * (n - 1))
    upper_index = int(((upper_angle + 135) / 270) * (n - 1))
    return distances[lower_index:upper_index+1]


# Door Detection Function
def detect_door_by_max_range(distances, max_val, min_region_length=5, tolerance=5):
    """
    Detect a continuous region in which the laser data values are close to max_val,
    and ensure that the values on both sides of this region are lower than (max_val - tolerance).
    
    Parameters:
        distances: List of laser data distances.
        max_val: The maximum distance value detected by the sensor, used to identify "open space".
        min_region_length: The minimum length of the continuous region.
        tolerance: If the difference between a data point and max_val is less than this value, it is considered "close to the maximum value".
    Returns:
        A list of the center angles.
    """
    n = len(distances)
    candidate_angles = []  # List to store the center angles of all candidate door regions.
    candidate_start = None  # Start index of the current candidate region.
    candidate_end = None    # End index of the current candidate region.

    for i in range(n):
        if abs(distances[i] - max_val) < tolerance:
            if candidate_start is None:
                candidate_start = i  # Record the start of the candidate region.
            candidate_end = i        # Update the candidate region's end index.
        else:
            if candidate_start is not None and (candidate_end - candidate_start + 1) >= min_region_length:
                left_idx = candidate_start - 1 if candidate_start > 0 else candidate_start
                right_idx = candidate_end + 1 if candidate_end < n - 1 else candidate_end
                if distances[left_idx] < max_val - tolerance and distances[right_idx] < max_val - tolerance:
                    mid_idx = (candidate_start + candidate_end) // 2
                    candidate_angles.append(index_to_angle(mid_idx, n))
            candidate_start = None
            candidate_end = None

    # Check the last candidate region if it exists and hasn't been processed.
    if candidate_start is not None and (candidate_end - candidate_start + 1) >= min_region_length:
        left_idx = candidate_start - 1 if candidate_start > 0 else candidate_start
        right_idx = candidate_end + 1 if candidate_end < n - 1 else candidate_end
        if distances[left_idx] < max_val - tolerance and distances[right_idx] < max_val - tolerance:
            mid_idx = (candidate_start + candidate_end) // 2
            candidate_angles.append(index_to_angle(mid_idx, n))
    return candidate_angles

# Static Obstacle Avoidance Strategy
def decide_movement_with_turn(distances, safe_distance=30):
    """
    Determine the robot's movement:
      - If the minimum distance in the front region is greater than safe_distance, move forward.
      - Otherwise, compute the average distances on the left and right sides and choose to turn toward the more open side.
      - If both sides are too close, then stop.
      
    Parameters:
        distances: List of laser data distances.
        safe_distance: The safety distance threshold.
    Returns:
        One of the commands: "FORWARD", "TURN_LEFT", "TURN_RIGHT", or "STOP".
    """
    n = len(distances)
    if n == 0:
        return "STOP"
    window = max(5, n // 50)
    center_index = n // 2
    front_region = distances[max(center_index - window, 0):min(center_index + window, n)]
    front_min = min(front_region) if front_region else 999
    if front_min > safe_distance:
        return "FORWARD"
    else:
        left_start = min(center_index + window, n-1)
        left_end = min(center_index + 3*window, n)
        left_region = distances[left_start:left_end]
        left_avg = np.mean(left_region) if len(left_region) > 0 else 0
        right_start = max(center_index - 3*window, 0)
        right_end = max(center_index - window, 0)
        right_region = distances[right_start:right_end]
        right_avg = np.mean(right_region) if len(right_region) > 0 else 0
        if left_avg < 10 and right_avg < 10:
            return "STOP"
        elif left_avg > right_avg:
            return "TURN_LEFT"
        else:
            return "TURN_RIGHT"

def execute_movement(robot, action):
    """
    Execute the corresponding movement based on the action command.
    
    Parameters:
        robot: The robot.
        action: The movement action command.
    """
    if action == "FORWARD":
        robot.Move(90, 90)
    elif action == "TURN_LEFT":
        robot.Move(-90, 90)
    elif action == "TURN_RIGHT":
        robot.Move(90, -90)
    else:
        robot.Move(0, 0)

# Stuck Detection and Recovery Strategy
class StuckDetector:
    """
    Detect if the robot is stuck by monitoring changes in its position.
    If the robot moves less than a threshold within a period, it is considered to be stuck.
    """
    def __init__(self, stuck_time=3.0):
        self.stuck_time = stuck_time 
        self.last_move_time = time.time()
        self.prev_pos = None

    def update(self, robot_pos):
        now = time.time()
        if self.prev_pos is not None:
            dx = robot_pos[0] - self.prev_pos[0]
            dy = robot_pos[1] - self.prev_pos[1]
            dist_moved = np.hypot(dx, dy)
            if dist_moved > 0.02: 
                self.last_move_time = now
        self.prev_pos = robot_pos
        return now - self.last_move_time > self.stuck_time


def stuck_recovery(robot, sensor):
    """
    When stuck is detected, execute a recovery strategy:
    The robot moves backward slightly, then checks the laser sensor data to find which side (left or right)
    has the closest obstacle. It then turns toward the opposite side and moves a short distance.
    
    Parameters:
        robot: The robot object.
        sensor: The LiDAR sensor object to retrieve distance data.
    """
    print("Stuck recovery")
    # Move back.
    robot.Move(-90, -90)
    time.sleep(1.0)
    # robot.Move(0, 0)
    
    distances = sensor.getAllDistances()
    if not distances:
        chosen_direction = random.choice(["LEFT", "RIGHT"])
    else:
        # split into left and right.
        n = len(distances)
        mid_index = n // 2
        left_distances = distances[:mid_index]  
        right_distances = distances[mid_index:] 
        
        # Find the minimum distance in each half.
        min_left = min(left_distances) if left_distances else float('inf')
        min_right = min(right_distances) if right_distances else float('inf')
        
        # If the closest obstacle is on the left, choose to go right; otherwise, go left.
        if min_left > min_right:
            chosen_direction = "RIGHT"
        else:
            chosen_direction = "LEFT"
    
    if chosen_direction == "LEFT":
        robot.Move(-90, 90)
    else:
        robot.Move(90, -90)
    time.sleep(1.0)
    robot.Move(90, 90)
    time.sleep(1.5)
    robot.Move(0, 0)


# Rotation Function
def rotate_to_angle(robot, current_angle, target_angle, speed=60):
    """
    Rotate the robot from its current angle to the target angle.
    
    Parameters:
        robot: The robot.
        current_angle: The current angle.
        target_angle: The target angle.
        speed: The speed during rotation.
    """
    delta = target_angle - current_angle
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360
    if delta > 0:
        robot.Move(-speed, speed)
    else:
        robot.Move(speed, -speed)
    time.sleep(abs(delta) / speed)
    # robot.Move(0, 0)

# Door Passing Function
def pass_through_door(robot, sensor, 
                      max_distance_value=399,
                      safe_distance=30,
                      angle_tolerance=10,
                      door_move_timeout=100.0,
                      step_time=1.0):
    """
    Move forward in small steps while performing real-time detection of the door and obstacles.
    When it is detected that the door is no longer in front or a timeout occurs, we assumed that the door has been successfully passed.
    
    Parameters:
        robot: The robot.
        sensor: The LiDAR sensor.
        max_distance_value: The maximum measurement distance of the LiDAR.
        safe_distance: The safety distance threshold.
        angle_tolerance: If the door candidate angle is less than this value, it is considered aligned.
        door_move_timeout: The maximum allowed duration for the door passing process.
        step_time: The duration for each small forward movement step.
    """
    start_time = time.time()
    not_in_front_count = 0  # Count consecutive frames where the door is not detected or is behind.
    required_not_in_front_frames = 3  # Number of consecutive frames required to consider that the door is no longer in front.

    while (time.time() - start_time) < door_move_timeout:
        distances = sensor.getAllDistances()
        if not distances:
            time.sleep(0.1)
            continue

        # Check if there are obstacles in the front region (the area within -90° to +90°).
        front_region = get_front_region(distances, lower_angle=-90, upper_angle=90)
        if min(front_region) < safe_distance:
            print("Obstacle detected in front during door passing, perform local obstacle avoidance")
            n = len(distances)
            center_index = n // 2
            window = max(5, n // 10)
            # Calculate the average distances for the left and right regions.
            left_start = min(center_index + window, n-1)
            left_end = min(center_index + 3 * window, n)
            left_region = distances[left_start:left_end]
            left_avg = np.mean(left_region) if len(left_region) > 0 else 0

            right_start = max(center_index - 3 * window, 0)
            right_end = max(center_index - window, 0)
            right_region = distances[right_start:right_end]
            right_avg = np.mean(right_region) if len(right_region) > 0 else 0

            if left_avg > right_avg:
                print("Left side is more open, turning left and moving forward a bit")
                robot.Move(-45, 45)
                time.sleep(0.5)
                robot.Move(90, 90)
                time.sleep(step_time)
                # robot.Move(0, 0)
            else:
                print("Right side is more open, turning right and moving forward a bit")
                robot.Move(45, -45)
                time.sleep(0.5)
                robot.Move(90, 90)
                time.sleep(step_time)
                # robot.Move(0, 0)
            continue 

        # Detect door candidate angles.
        door_angles = detect_door_by_max_range(distances, max_distance_value, min_region_length=5, tolerance=5)
        # print("door_angles:", door_angles)
        if door_angles:
            # Select valid candidates.
            valid_candidates = [angle for angle in door_angles if abs(angle) < 100]
            if valid_candidates:
                door_angle = valid_candidates[0]
            else:
                door_angle = None
        else:
            door_angle = None

        # If a door is detected and its angle is within a reasonable range.
        if door_angle is not None and abs(door_angle) <= 100:
            not_in_front_count = 0  
            if abs(door_angle) > angle_tolerance:
                current_ori = robot.sim_actuator.getObjectOrientation(robot.simrobot, -1)[2]
                current_angle_deg = current_ori * 180.0 / np.pi
                target_heading = current_angle_deg + door_angle
                rotate_to_angle(robot, current_angle_deg, target_heading, speed=60)
                robot.Move(90, 90)
                time.sleep(step_time)
                # robot.Move(0, 0)
            else:
                # Door is straight ahead
                robot.Move(200, 200)
                time.sleep(step_time)
                # robot.Move(0, 0)
        else:
            not_in_front_count += 1
            robot.Move(90, 90)
            time.sleep(step_time)
            # robot.Move(0, 0)

        # Exit condition: if the door is not detected (or is not in front) for consecutive frames.
        if not_in_front_count >= required_not_in_front_frames:
            # Door no longer in front.
            return

    print("Exceeded maximum door passing time.")

# Main Program
if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    robot = PioneerP3DX('PioneerP3DX', client=client)
    sensor = SickTIM310('./SickTIM310', client=client)

    stuck_detector = StuckDetector(stuck_time=3.0)

    start_time = time.time()
    max_distance_value = 399    # Maximum LiDAR detection distance.
    angle_tolerance = 10        # Door angle tolerance.
    safe_distance = 30          # Safety distance threshold.

    try:
        while True:
            distances = sensor.getAllDistances()
            if not distances:
                continue

            pos = robot.sim_actuator.getObjectPosition(robot.simrobot, -1)
            if stuck_detector.update((pos[0], pos[1])):
                stuck_recovery(robot,sensor)
                print("Stuck recovery completed")
                continue

            # Obstacle avoidance.
            front_region = get_front_region(distances, lower_angle=-60, upper_angle=60)
            if min(front_region) < safe_distance:
                print("Obstacle detected in the front region, perform obstacle avoidance")
                action = decide_movement_with_turn(front_region, safe_distance=safe_distance)
                execute_movement(robot, action)
                time.sleep(0.5)
                # robot.Move(0, 0)
                continue

            # Detect door candidate regions.
            door_candidates = detect_door_by_max_range(distances, max_distance_value, min_region_length=5, tolerance=5)
            if door_candidates:
                # Select valid candidates.
                valid_candidates = [angle for angle in door_candidates if abs(angle) < 100]
                if valid_candidates:
                    door_candidate = valid_candidates[0]
                    if abs(door_candidate) > 100:
                        print("Door angle exceeds ±100°, ignore")
                    else:
                        if abs(door_candidate) < angle_tolerance:
                            start_door_pos = robot.sim_actuator.getObjectPosition(robot.simrobot, -1)
                            pass_through_door(robot, sensor,
                                              max_distance_value=max_distance_value,
                                              safe_distance=safe_distance,
                                              angle_tolerance=angle_tolerance,
                                              door_move_timeout=40.0,
                                              step_time=0.3)
                        else:
                            current_ori = robot.sim_actuator.getObjectOrientation(robot.simrobot, -1)[2]
                            current_angle_deg = current_ori * 180.0 / np.pi
                            target_heading = current_angle_deg + door_candidate
                            rotate_to_angle(robot, current_angle_deg, target_heading, speed=60)
                else:
                    action = decide_movement_with_turn(distances, safe_distance=safe_distance)
                    execute_movement(robot, action)
            else:
                # If no door candidate regions are detected, continue moving forward while avoiding obstacles.
                action = decide_movement_with_turn(distances, safe_distance=safe_distance)
                execute_movement(robot, action)

    except Exception as e:
        print("Exception:", e)
    finally:
        robot.Move(0, 0)
        sim.stopSimulation()
        print("Program terminated")




