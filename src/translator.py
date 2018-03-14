import math


class Translator(object):
    """Class for translating truck wheel angle, speed, and angular velocity
    to the corrseponding pwm values. """
    def __init__(self):
        # TODO: speed_pwm and alpha_pwm can be local variables.
        self.speed_pwm = 1500   # pwm value for the speed.
        self.alpha_pwm = 1500   # pwm value for the wheel angle alpha.

        # List for pwm signals and corresponding turning radius for left and right turns.
        self.right_radius = [[1200, 0.7172], [1250, 0.7849], [1300, 0.8633], [1350, 1.1609],
                             [1400, 1.3933], [1450, 3], [1500, 20]]
        self.left_radius = [[1500, 20], [1550, 3], [1600, 1.7138], [1650, 1.4199], [1700, 1.1382],
                            [1750, 1.0422], [1800, 1.0417]]
        self.right_radius.sort(key=lambda x: x[1])
        self.left_radius.sort(key=lambda x: x[1])

        # List containing measurements of speed pwms and resulting speeds.
        self.speeds = [[1500, 0], [1450, 0.89], [1400, 1.97], [1350, 2.75]]
        self.speeds.sort(key=lambda x: x[1])

        self.speed_pwm_min = 1100       # Minimum value for speed_pwm.
        self.speed_pwm_max = 1500       # Maximum value for speed_pwm.

        self.alpha_pwm_min = 1100       # Minimum wheel angle pwm.
        self.alpha_pwm_max = 1900       # Maximum wheel angle pwm.

    def _translate_omega(self, speed, omega):
        """Calculates a steering pwm value from the given angular and linear velocities. """
        # Calculate turning radius.
        try:
            radius = abs(speed / omega)
        except ZeroDivisionError:
            radius = 100

        # Check if left or right turn.
        if omega > 0:
            radius_list = self.left_radius
        else:
            radius_list = self.right_radius

        length = len(radius_list)

        # Find the lowest index for which r is smaller than the radius value.
        i = 0
        while i < length and radius > radius_list[i][1]:
            i += 1

        # If at first or last value use that value.
        if i <= 0:
            self.alpha_pwm = radius_list[0][0]
        elif i >= length:
            self.alpha_pwm = radius_list[length - 1][0]
        else:             # Calculate angle pwm using straight line equation.
            lower = i - 1
            upper = i

            k = (radius_list[upper][0] - radius_list[lower][0]) / (
                    radius_list[upper][1] - radius_list[lower][1])

            self.alpha_pwm = int(radius_list[lower][0] + (radius - radius_list[lower][1]) * k)

        # Make sure that the translated pwm is within the bounds.
        if self.alpha_pwm > self.alpha_pwm_max:
            self.alpha_pwm = self.alpha_pwm_max
        if self.alpha_pwm < self.alpha_pwm_min:
            self.alpha_pwm = self.alpha_pwm_min

    def _translate_speed(self, speed):
        """Calculates the pwm value corresponding to the desired speed v.
        Interpolates linearly from a list of measurements. """
        length = len(self.speeds)

        if length < 2:
            print('Not enough measurements to translate speed input.')
            self.speed_pwm = 1500
            return

        # Find the lowest index for which v is smaller than the speed value.
        i = 0
        while i < length and speed > self.speeds[i][1]:
            i += 1

        # Get the lower and upper indices that will be used for line equation.
        if i <= 0:
            lower = 0
            upper = 1
        elif i >= length:
            lower = length - 2
            upper = length - 1
        else:
            lower = i - 1
            upper = i

        # Calculate speed pwm using straight line equation.
        k = (self.speeds[upper][0] - self.speeds[lower][0]) / (
            self.speeds[upper][1] - self.speeds[lower][1])

        self.speed_pwm = int(self.speeds[lower][0] + (speed - self.speeds[lower][1]) * k)

        # Make sure that the translated speed is within the bounds.
        if self.speed_pwm > self.speed_pwm_max:
            self.speed_pwm = self.speed_pwm_max
        if self.speed_pwm < self.speed_pwm_min:
            self.speed_pwm = self.speed_pwm_min

    def get_speed_value(self, speed):
        """Returns the pwm speed that corresponds to the speed v. """
        self._translate_speed(speed)
        return self.speed_pwm

    def get_omega_value(self, omega, speed):
        """Returns the pwm angle that corresponds to given speed v and angular
        velocity w. """
        self._translate_omega(speed, omega)
        return self.alpha_pwm














