import time
from modules import cbpi
from modules.core.controller import KettleController
from modules.core.controller import KettleController
from modules.core.props import Property

@cbpi.controller
class CascadePID(KettleController):
    a_inner_sensor = Property.Sensor(label="Inner loop sensor")
    b_inner_kp = Property.Number("Inner loop proportional term", True, 5.0)
    c_inner_ki = Property.Number("Inner loop integral term", True, 0.25)
    d_inner_kd = Property.Number("Inner loop derivative term", True, 0.0)
    e_inner_integrator_max = Property.Number("Inner loop integrator max", True, 15.0)
    f_outer_kp = Property.Number("Outer loop proportional term", True, 0.0)
    g_outer_ki = Property.Number("Outer loop integral term", True, 2.0)
    h_outer_kd = Property.Number("Outer loop derivative term", True, 1.0)
    i_outer_integrator_max = Property.Number("Outer loop integrator max", True, 15.0)
    j_update_interval = Property.Number("Update interval", True, 2.5)

    def stop(self):
        self.heater_off()
        super(KettleController, self).stop()

    def run(self):
        # Error check
        if float(self.j_update_interval) <= 0.0:
            self.notify("PID Error", "Update interval must be positive", timeout=None, type="danger")
            raise ValueError("PID - Update interval must be positive")
        if float(self.e_inner_integrator_max) < 0.0:
            self.notify("PID Error", "Inner loop max integrator must be >= 0", timeout=None, type="danger")
            raise ValueError("PID - Inner loop max integrator must be >= 0")
        if float(self.i_outer_integrator_max) < 0.0:
            self.notify("PID Error", "Outer loop max integrator must be >= 0", timeout=None, type="danger")
            raise ValueError("PID - Outer loop max integrator must be >= 0")
        elif not isinstance(self.a_inner_sensor, unicode):
            self.notify("PID Error", "An inner sensor must be selected", timeout=None, type="danger")
            raise UserWarning("PID - An inner sensor must be selected")
        else:
            self.heater_on(0.0)

        # Get inner sensor as an integer
        inner_sensor = int(self.a_inner_sensor)

        # Ensure all numerical properties are floats
        inner_kp = float(self.b_inner_kp)
        inner_ki = float(self.c_inner_ki)
        inner_kd = float(self.d_inner_kd)
        inner_integrator_max = float(self.e_inner_integrator_max)
        outer_kp = float(self.f_outer_kp)
        outer_ki = float(self.g_outer_ki)
        outer_kd = float(self.h_outer_kd)
        outer_integrator_max = float(self.i_outer_integrator_max)
        update_interval = float(self.j_update_interval)

        # Initialize PID cascade
        if cbpi.get_config_parameter("unit", "C") == "C":
            outer_pid = PID(outer_kp, outer_ki, outer_kd, 0.0, 100.0, outer_integrator_max)
        else:
            outer_pid = PID(outer_kp, outer_ki, outer_kd, 32, 212, outer_integrator_max)

        inner_pid = PID(inner_kp, inner_ki, inner_kd, 0.0, 100.0, inner_integrator_max)

        while self.is_running():
            waketime = time.time() + update_interval

            # Get the target temperature
            outer_target_value = self.get_target_temp()

            # Calculate inner target value from outer PID
            outer_current_value = self.get_temp()
            inner_target_value = round(outer_pid.update(outer_current_value, outer_target_value),2)

            # Calculate inner output from inner PID
            inner_current_value = float(cbpi.cache.get("sensors")[inner_sensor].instance.last_value)
            inner_output = round(inner_pid.update(inner_current_value, inner_target_value), 2)

            # Update the heater power
            self.actor_power(inner_output)

            # Print loop details
            print("[%s] Outer loop PID target/actual/output: %s/%s/%s" % (waketime, outer_target_value, outer_current_value, inner_target_value))
            print("[%s] Inner loop PID target/actual/output: %s/%s/%s" % (waketime, inner_target_value, inner_current_value, inner_output))

            # Sleep until update required again
            if waketime <= time.time():
                self.notify("PID Error", "Update interval is too short to complete calculations", timeout=None, type="danger")
                raise ValueError("PID - Update interval is too short to complete CascadePID calculations")
            else:
                self.sleep(waketime - time.time())


@cbpi.controller
class SinglePID(KettleController):
    a_kp = Property.Number("Proportional term", True, 10.0)
    b_ki = Property.Number("Integral term", True, 2.0)
    c_kd = Property.Number("Derivative term", True, 1.0)
    d_output_max = Property.Number("Output max", True, 100.0)
    e_output_min = Property.Number("Output min", True, 0.0)
    f_integrator_max = Property.Number("Integrator max", True, 15.0)
    g_update_interval = Property.Number("Update interval", True, 2.5)

    def stop(self):
        self.heater_off()
        super(KettleController, self).stop()

    def run(self):
        # Error check
        if float(self.g_update_interval) <= 0.0:
            self.notify("PID Error", "Update interval must be positive", timeout=None, type="danger")
            raise ValueError("PID - Update interval must be positive")
        elif float(self.f_integrator_max) < 0.0:
            self.notify("PID Error", "Max integrator must be >= 0", timeout=None, type="danger")
            raise ValueError("PID - Max integrator must be >= 0")
        elif float(self.d_output_max) <= float(self.e_output_min):
            self.notify("PID Error", "Output maxmimum must be greater than minimum", timeout=None, type="danger")
            raise ValueError("PID - Output maxmimum must be greater than minimum")
        else:
            self.heater_on(0.0)

        # Ensure all numerical properties are floats
        kp = float(self.a_kp)
        ki = float(self.b_ki)
        kd = float(self.c_kd)
        output_max = float(self.d_output_max)
        output_min = float(self.e_output_min)
        integrator_max = float(self.f_integrator_max)
        update_interval = float(self.g_update_interval)

        # Initialize PID
        SinglePID = PID(kp, ki, kd, output_min, output_max, integrator_max)

        while self.is_running():
            waketime = time.time() + update_interval

            # Get the target temperature
            target_value = self.get_target_temp()

            # Calculate inner target value from outer PID
            current_value = self.get_temp()
            output = round(SinglePID.update(current_value, target_value), 2)

            # Update the heater power
            self.actor_power(output)

            # Print details
            print("[%s] PID target/actual/output: %s/%s/%s" % (waketime, target_value, current_value, output))

            # Sleep until update required again
            if waketime <= time.time():
                self.notify("PID Error", "Update interval is too short to complete calculations", timeout=None, type="danger")
                raise ValueError("PID - Update interval is too short to complete calculations")
            else:
                self.sleep(waketime - time.time())


class PID(object):
    def __init__(self, kp, ki, kd, output_min, output_max, integrator_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integrator_max = integrator_max
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
            return max(min(self.kp * current_error, self.output_max), self.output_min)

        # Regular interation
        else:

            # Calculate duration of interation
            current_time = time.time()
            iteration_time = current_time - self.last_time
            self.last_time = current_time

            # Calculate error
            current_error = target - current

            # Integrate error (respecting bounds to reduce integrator windup)
            self.integrator = max(min(self.integrator + (current_error * iteration_time), self.integrator_max), -self.integrator_max)

            # Calculate error derivative
            derivative = (current_error - self.last_error)/iteration_time

            # Calculate output
            p_action =  self.kp * current_error
            i_action = self.ki * self.integrator
            d_action = self.kd * derivative

            # Update last_error
            self.last_error = current_error

            # Return output
            return max(min(p_action + i_action + d_action, self.output_max), self.output_min)
