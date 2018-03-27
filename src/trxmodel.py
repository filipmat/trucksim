"""
Modules for modeling of the vehicle.
"""

import math

axle_length = 0.33                      # Length between wheel pairs.

# Values for old linear model. Not correct for trx vehicle.
zero_angle_input = 1500                 # Input signal for which the wheel angle is zero.
zero_velocity_input = 1500              # Input signal for which the velocity is zero.
angle_input_factor = 300/(math.pi/6)    # y/x: a change of y input gives x radians wheel angle.
speed_input_factor = 100./(-1)          # y/x: a change of y input gives x m/s velocity.

throttle_min = 1500
throttle_max = 2100

angle_to_steering_left_k = [1459, -87.2, -1633]
angle_to_steering_right_k = [1642, -145, 1274]
speed_to_throttle_k = [1576, 33.4, 137]

steering_to_angle_left_k = [-1.25217, 0.003609, -0.0000018607]
steering_to_angle_right_k = [6.876317, -0.006896, 0.0000016103]
throttle_to_speed_k = [-30.2286483, 0.031891142, -0.000008005022786]    # For gear 0.

# TODO: go from inputs to velocity/wheel angle (for simulation).
# TODO: handle different gears.


def wheel_angle_to_steering_input(wheel_angle):
    """Returns the steering input corresponding to the wheel angle. """
    steering_input = 0

    if wheel_angle > 0:
        coefficients = angle_to_steering_left_k
    else:
        coefficients = angle_to_steering_right_k

    for i, k in enumerate(coefficients):
        steering_input += k*wheel_angle**i

    #steering_input = wheel_angle * angle_input_factor + zero_angle_input

    return steering_input


def steering_input_to_wheel_angle(steering_input):
    """Returns the wheel angle corresponding to the steering input signal. """
    wheel_angle = 0

    if steering_input < 1500:
        coefficients = steering_to_angle_left_k
    else:
        coefficients = steering_to_angle_right_k

    for i, k in enumerate(coefficients):
        wheel_angle += k*steering_input**i

    #wheel_angle = (steering_input - zero_angle_input) / angle_input_factor

    return wheel_angle


def throttle_input_to_linear_velocity(throttle_input):
    """Returns the linear velocity corresponding to the input signal.
    Used by simulated vehicle. """
    if throttle_input <= 1500:
        return 0

    if throttle_input > throttle_max:
        throttle_input = throttle_max

    linear_velocity = 0

    for i, k in enumerate(throttle_to_speed_k):
        linear_velocity += k*throttle_input**i

    if linear_velocity < 0:
        return 0

    #linear_velocity = (throttle_input - zero_velocity_input) / speed_input_factor

    return linear_velocity


def linear_velocity_to_throttle_input(linear_velocity):
    """Returns the throttle input corresponding to the linear velocity.
    Used by controller. """
    throttle_input = 0

    for i, k in enumerate(speed_to_throttle_k):
        throttle_input += k*linear_velocity**i

    #throttle_input = linear_velocity * speed_input_factor + zero_velocity_input

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
