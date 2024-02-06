import collections
import math

import numpy as np
import wpimath.units
from wpimath.geometry import Pose3d, Rotation3d


def fit_to_boundaries(value, minimum_value=None, maximum_value=None):
    """Fits a value to boundaries. None to dismiss a boundary."""
    if minimum_value is not None:
        value = max(minimum_value, value)
    if maximum_value is not None:
        value = min(maximum_value, value)
    return value


def square_input(input):
    """Retourne la valeur au carré en conservant le signe"""
    return math.copysign(input * input, input)


def map_value(value, old_min, old_max, new_min, new_max):
    # Figure out how 'wide' each range is
    old_span = old_max - old_min
    new_span = new_max - new_min

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - old_min) / float(old_span)

    # Convert the 0-1 range into a value in the right range.
    return new_min + (valueScaled * new_span)


def rotate_vector(vector, angle):
    rad = math.radians(angle)
    x = vector[0] * math.cos(rad) - vector[1] * math.sin(rad)
    y = vector[0] * math.sin(rad) + vector[1] * math.cos(rad)
    return (x, y)


def calculate_optimal_launch_angle(
    distance, height_difference, initial_velocity
) -> wpimath.units.degrees | None:
    """
    Calculates the optimal launch angle to hit a target at a given distance and height difference.

    :param distance: The horizontal distance to the target (meters).
    :param height_difference: The height difference to the target (meters).
    :param initial_velocity: The initial velocity of the disk (meters/second).
    :return: The optimal launch angle (degrees) to hit the target.
    """
    angle = math.degrees(math.atan(height_difference / distance))
    return angle

    # TODO Make it better?
    g = 9.81  # Acceleration due to gravity (m/s^2)
    v = initial_velocity

    # The quadratic formula components (a, b, and c) are used in solving for the angle.
    a = g * distance**2
    b = 2 * height_difference * v**2 - 2 * g * distance**2
    c = v**4 - g * (g * distance**2 + 2 * height_difference * v**2)

    # Check if the discriminant is positive to ensure a real solution exists.
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return None

    # Calculate the two possible angles and select the smaller one for the optimal trajectory.
    theta1 = math.atan2(v**2 + math.sqrt(discriminant), g * distance)
    theta2 = math.atan2(v**2 - math.sqrt(discriminant), g * distance)

    # Convert the optimal angle to degrees.
    angle_deg = math.degrees(min(theta1, theta2))

    return wpimath.units.degrees(angle_deg)


def compute_angle(x, y):
    """Returns the absolute angle (in degrees) based on a vector"""
    angle_radians = math.atan2(y, x)
    angle_degrees = math.degrees(angle_radians)
    return angle_degrees


def filter_and_average(values):
    """
    Averages values while filtering out extreme values defined as more than 3 standard deviations from the mean.

    Parameters:
    - values: A list of numeric values representing the time series data.

    Returns:
    - The average of the filtered values, or None if all values are filtered out.
    """
    # Convert the list of values to a numpy array
    data = np.array(values)

    # Calculate mean and standard deviation
    mean = np.mean(data)
    std = np.std(data, ddof=1)  # ddof=1 for sample standard deviation

    # Filter out extreme values (3times standard deviation?)
    filtered_data = data[np.abs(data - mean) <= std]

    # Calculate and return the average of the filtered data
    if len(filtered_data) > 0:
        return np.mean(filtered_data)
    else:
        return None  # Return None if all values are filtered out


class VisionFilter:
    def __init__(self):
        self.x = np.full(10, np.inf, dtype=float)
        self.y = np.full(10, np.inf, dtype=float)
        self.z = np.full(10, np.inf, dtype=float)
        self.rot = np.full(10, np.inf, dtype=float)
        self.idx = 0
        self.last_time = 0

    def update(self, pose: Pose3d, time) -> Pose3d | None:
        # It's been too long, reset the value
        if time > self.last_time + 500 or self.x[0] == np.inf:
            self.x.fill(pose.x)
            self.y.fill(pose.y)
            self.z.fill(pose.z)
            self.rot.fill(pose.rotation().z)

        self.x[self.idx] = pose.x
        self.y[self.idx] = pose.y
        self.z[self.idx] = pose.z
        self.rot[self.idx] = pose.rotation().z
        self.idx = (self.idx + 1) % 10
        average_pose = self.filter_and_average_pose()
        if average_pose is None:
            return None
        return Pose3d(
            average_pose[0],
            average_pose[1],
            average_pose[2],
            Rotation3d(0, 0, average_pose[3]),
        )

    def filter_and_average_pose(self) -> None | tuple[float, float, float, float]:
        x = filter_and_average(self.x)
        y = filter_and_average(self.y)
        z = filter_and_average(self.z)
        rot = filter_and_average(self.rot)
        if x is None or y is None or rot is None:
            return None
        return x, y, z, rot


#
# if __name__ == "__main__":
#     data = [
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(1, 2, 5, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(2, 3, 6, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(100, 200, 300, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(10, 4, 7, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(10, 4, 7, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(10, 4, 7, Rotation3d(0, 0, 0)), 0),
#         (Pose3d(10, 4, 7, Rotation3d(0, 0, 0)), 0),
#     ]
#     filter = VisionFilter()
#     for pose in data:
#         print(filter.update(pose[0], pose[1]))