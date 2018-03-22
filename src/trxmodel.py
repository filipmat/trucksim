"""
Modules for modeling of the vehicle.
"""

import math

axle_length = 0.33                      # Length between wheel pairs.
zero_angle_input = 1500                 # Input signal for which the wheel angle is zero.
zero_velocity_input = 1500              # Input signal for which the velocity is zero.
angle_input_factor = 300/(math.pi/6)    # y/x: a change of y input gives x radians wheel angle.
speed_input_factor = 100./(-1)          # y/x: a change of y input gives x m/s velocity.

# TODO: Calculate angle_input_factor and speed_input_factor from measurements.
# In current model wheel_angle_to_steering_input and inverse is enough to go from angular_velocity
# to steering_input and vice versa. Possible that it depends on left/right turn.


def wheel_angle_to_steering_input(wheel_angle):
    """Returns the steering input corresponding to the wheel angle. """
    steering_input = wheel_angle * angle_input_factor + zero_angle_input

    return steering_input


def steering_input_to_wheel_angle(steering_input):
    """Returns the wheel angle corresponding to the steering input signal. """
    wheel_angle = (steering_input - zero_angle_input) / angle_input_factor

    return wheel_angle


def throttle_input_to_linear_velocity(throttle_input):
    """Returns the linear velocity corresponding to the input signal.
    Used by simulated vehicle. """
    linear_velocity = (throttle_input - zero_velocity_input) / speed_input_factor

    return linear_velocity


def linear_velocity_to_throttle_input(linear_velocity):
    """Returns the throttle input corresponding to the linear velocity.
    Used by controller. """
    throttle_input = linear_velocity * speed_input_factor + zero_velocity_input

    return throttle_input


def wheel_angle_to_angular_velocity(wheel_angle, linear_velocity):
    """Returns the angular velocity corresponding to the wheel angle at the given linear
    velocity."""
    angular_velocity = linear_velocity/axle_length * math.tan(wheel_angle)

    return angular_velocity


def angular_velocity_to_wheel_angle(angular_velocity, linear_velocity):
    """Returns the wheel angle corresponding to the desired angular velocity at the given
    linear velocity. """
    try:
        wheel_angle = math.atan(angular_velocity*axle_length/linear_velocity)
    except ZeroDivisionError:
        wheel_angle = 0     # No turning if vehicle is standing still.

    return wheel_angle


def angular_velocity_to_steering_input(angular_velocity, linear_velocity):
    """Returns the steering input corresponding to the desired angular velocity at the given
    linear velocity.
    Used by controller. """
    wheel_angle = angular_velocity_to_wheel_angle(angular_velocity, linear_velocity)
    steering_input = wheel_angle_to_steering_input(wheel_angle)

    return steering_input


def steering_input_to_angular_velocity(steering_input, linear_velocity):
    """Returns the angular velocity corresponding to the steering input at the given linear
    velocity.
    Used by simulated vehicle. """
    wheel_angle = steering_input_to_wheel_angle(steering_input)
    angular_velocity = wheel_angle_to_angular_velocity(wheel_angle, linear_velocity)

    return angular_velocity


def sign(x):
    """Returns the sign of x. """
    return 1 if x >= 0 else - 1
