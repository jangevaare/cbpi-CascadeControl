import time
from modules import cbpi
from modules.core.controller import KettleController
from modules.core.props import Property

@cbpi.controller
class CascadePID(KettleController):
    inner_sensor = Property.Sensor(label="Inner loop sensor")
    inner_kp = Property.Number("Inner loop proportional term", True, 10.0)
    inner_ki = Property.Number("Inner loop integral term", True, 0.0)
    inner_kd = Property.Number("Inner loop derivative term", True, 0.0)
    outer_kp = Property.Number("Outer loop proportional term", True, 30.0)
    outer_ki = Property.Number("Outer loop integral term", True, 0.0)
    outer_kd = Property.Number("Outer loop derivative term", True, 0.0)
    update_interval = Property.Number("Update interval", True, 2.0)

    def __init__(self):
        if self.update_interval <= 0:
            raise ValueError("Period, must be positive")

    def stop(self):
        super(KettleController, self).stop()
        self.heater_off()

    def run(self):
        self.heater_on(0.0)
        # Get the target temperature
        outer_target_value = self.get_target_temp()

        # Ensure all terms are floats
        inner_kp = float(self.inner_kp)
        inner_ki = float(self.inner_ki)
        inner_kd = float(self.inner_kd)
        outer_kp = float(self.outer_kp)
        outer_ki = float(self.outer_ki)
        outer_kd = float(self.outer_kd)

        # Get inner sensor as integer
        inner_sensor = int(self.inner_sensor)

        # Set a max for the the integrators
        integrator_max = 15.0

        # Initialize PID cascade
        if self.get_config_parameter("unit", "C") == "C":
            outer_pid = PID(outer_kp, outer_ki, outer_kd, 0.0, 100.0, integrator_max)
        else:
            outer_pid = PID(outer_kp, outer_ki, outer_kd, 32, 212, integrator_max * 1.8)

        inner_pid = PID(inner_kp, inner_ki, inner_kd, 0.0, 100.0, 15)

        while self.is_running():
            waketime = time.time() + float(self.update_interval)
            # Calculate target value of inner loop, as the output of the outer loop
            outer_current_value = self.get_temp()
            # Calculate output of inner loop actor based on current and target values
            inner_current_value = float(cbpi.cache.get("sensors")[inner_sensor].instance.last_value)
            inner_output = update(inner_pid, inner_current_value, inner_target_value)
            # Update the heater power
            self.actor_power(inner_output)
            # Sleep until update required again
            if waketime <= time.time():
                self.notify("cascadePID Error", "Update interval is too short complete calculations", timeout=None, type="danger")
                raise ValueError("Update interval is too short to complete cascadePID calculations")
            else:
                self.sleep(waketime - time.time())

class PID(object):
    def __init__(self, kp, ki, kd, output_min, output_max, integrator_max):
        self.last_time = 0.0
        self.last_error = 0.0
        self.integrator = 0.0

    def update(self, current, target):
        # Initialization interation
        if self.last_time == 0.0:
            self.last_time = time.time()
            current_error = target - current
            # Update last_error
            self.last_error = current_error
            # Return output
            return max(min(self.kp * current_error, output_max), output_min)
        # Regular interation
        else:
            # Calculate duration of interation
            current_time = time.time()
            interation_time = current_time - last_time
            self.last_time = current_time
            # Calculate error
            current_error = target - current
            # Integrate error (respecting bounds to reduce integrator windup)
            self.integrator = max(min(self.integrator + (current_error * interation_time), self.integrator_max), -self.integrator_max)
            # Calculate error derivative
            derivative = (current_error - self.last_error)/interation_time
            # Calculate output
            p_action =  self.kp * current_error
            i_action = self.ki * self.integral
            d_action = self.kd * derivative
            # Update last_error
            self.last_error = current_error
            # Return output
            return max(min(p_action + i_action + d_action, output_max), output_min)
