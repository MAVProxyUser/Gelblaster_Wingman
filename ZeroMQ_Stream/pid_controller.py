class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint

        self._prev_error = 0
        self._integral = 0

        self.output_limits = output_limits

    def reset(self):
        self._prev_error = 0
        self._integral = 0

    def update(self, measured_value):
        error = self.setpoint - measured_value
        delta_error = error - self._prev_error

        self._integral += error
        derivative = delta_error

        # Calculate PID output
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        # Clip to output limits
        min_limit, max_limit = self.output_limits
        if min_limit is not None:
            output = max(output, min_limit)
        if max_limit is not None:
            output = min(output, max_limit)

        self._prev_error = error
        return output

    def set_parameters(self, kp, ki, kd):
        """
        Update the PID parameters.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

if __name__ == "__main__":
    # Test the PID controller
    pid = PIDController(1, 0.1, 0.01, setpoint=50)
    test_values = [40, 45, 48, 50, 51, 53, 55]  # simulated measurements
    for val in test_values:
        output = pid.update(val)
        print(f"Measured: {val}, Output: {output}")


