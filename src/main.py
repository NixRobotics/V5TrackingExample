# EXAMPLE CODE USING INERTIAL SENSOR AND TRACKING WHEELS FOR ROBOT ORIENTATION
#
# Note this example builds off of the V5GyroExample amd V5PIDExample projects
#  https://github.com/NixRobotics/V5GyroExample
#  https://github.com/NixRobotics/V5PIDExample
# To use this code you will need to have have determined the gyro_scale errro factor for your robot
#  and also tuned the PID constants for turning and driving for your robot
# 
# Tracking wheels are optional. If using tracking wheels, follow the steps for determining the actual offsets for the forward
# and sideways tracking wheels. If tracking wheels are not used then the code will use the motor encoders. This works fine as long
# as wheels do not slip

# Library imports
from vex import *
from math import sin, cos, radians, degrees

brain = Brain()

# DEVICE DECLARATIONS

# declare motors
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)

inertial = Inertial(Ports.PORT5)

# NOTE: GYRO_SCALE is used to compensate for each inertial sensor's built in error. This will be different for each sensor
# and must be determined experimentally before use.
# 
# See the V5GyroExample project for more details 
#
# IMPORTANT: Robot must turn cleanly for this calibration to be accurate if doing this here. Make sure at least the TurnPID
# funcionality is working before setting this and pay attention to any residual error reported by the sensor after a full turn.

ACTUAL_ROBOT_FULL_TURN = 361.0 # (~361-362 for CODE BOT) e.g. if robot actually turns 365 degrees for a 360 rotation enter 365 here
GYRO_SCALE_FOR_TURNS = 360.0 / ACTUAL_ROBOT_FULL_TURN
GYRO_SCALE_FOR_READOUT = ACTUAL_ROBOT_FULL_TURN / 360.0

# NOTE: Use ROBOT_INITIALIZED to allow movement. Calibration time is hidden when connected to field, but we need to prevent robot
#  from moving if we just do Program->Run on the controller
ROBOT_INITIALIZED = False

def pre_autonomous():
    # actions to do when the program starts
    global ROBOT_INITIALIZED

    # IMPORTANT: wait for sensors to initialize fully. Always include a small delay when using any sensors. This includes the 3-wire ports
    wait(0.1, SECONDS)

    # calibrate inertial and wait for completion - takes around 2 seconds
    # IMPORTANT: Robot must be stationary on a flat surface while this runs. Do not touch robot during calibration
    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    tracker_thread = Thread(Tracking.tracker_thread)
    wait(0.1, SECONDS) # allow some time for tracker to start

    ROBOT_INITIALIZED = True

# AUTONOMOUS HELPER FUNCTIONS

# The GyroHelper class will be depreacted here as all the required functionality is included in the Tracking class
# The main reason for deprecating this class is that we only want one source for heading readings. It gets messy when we want to
# reset headings based on alignining to walls or field elemements if we have to sync multiple sources of heading information 
class GyroHelper:
    # returns the inertial sensor's corrected direction as continuous ROTATION [-inf, +inf]
    # this is the only version of the direction routines that queries the inertial sensor directly
    @staticmethod
    def gyro_rotation():
        return inertial.rotation(DEGREES) * GYRO_SCALE_FOR_READOUT

    # performs modulus operation on the input so that output is in range [0, 360) degrees
    # note that this will lose history on total full revolutions, but useful if we want current HEADING of the robot
    # @param rotation as ROTATION value (either corrected or not - function is agnostic)
    @staticmethod
    def to_heading(rotation):
        return rotation % 360.0

    # performs modulus operation and offset on the input so that output is in range (-180, + 180] degrees
    # note that this will lose history on total full revolutions, but useful if we want current ANGLE of the robot
    # @param rotation as ROTATION value (either corrected or not - function is agnostic)
    @staticmethod
    def to_angle(rotation):
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle

    # returns the inertial sensor's corrected direction as HEADING [0, 360) degrees
    @staticmethod
    def gyro_heading():
        return GyroHelper.to_heading(GyroHelper.gyro_rotation())

    # returns the inertial sensor's corrected direction as ANGLE (-180, +180] degrees
    @staticmethod
    def gyro_angle():
        return GyroHelper.to_angle(GyroHelper.gyro_rotation())

    # Calculate a "raw" turn angle to get the robot facing towards a "real" HEADING based on current gyro reading
    #
    # This will return the smallest amount either left or right, ie no turns greater than 180deg. Provide own function if you want to turn
    # longer way around for some reason  e.g. 270degrees left instead of 90degrees right
    #
    # @param Input heading reflects the true HEADING we want the robot to finish at
    # Returns the scaled turn ANGLE with negative value up to -180deg * gyro_scale for left turn and positive value up to +180deg * scale_scale for right turn
    #
    # NOTE: The scaled return value in this case will *not* represent true motion of the robot, but rather the value we want from the gyro to get this motion
    # Therefore, returned value may exceed -180 to +180 degree range necessarily to compensate for a robot that underturns, so we apply the scale factor last,
    # meaning do not apply any additional limit code or bounds checking on the return value
    @staticmethod
    def calc_angle_to_heading(heading):
        # read corrected sensor as HEADING - this should reflect the robot's true HEADING, assuming scale factor is set correctly and sensor has not
        # drifted too much
        current_heading = GyroHelper.gyro_heading()
        # delta_heading will be the difference between the desired (real) heading and current (real) heading
        delta_heading = heading - current_heading
        # ensure result is in range -180deg (left turns) to +180deg (right turns) and finally multiply by scale factor
        delta_angle = GyroHelper.to_angle(delta_heading) * GYRO_SCALE_FOR_TURNS

        # returned value can be fed direcltly to drivetrain.turn_for(), but not drivetrain.turn_to_heading()
        return delta_angle

    # Computes the "raw" rotation value we want the gyro to read for a "real" HEADING
    # @param Input heading reflects the true HEADING we want the robot to finish at
    # Returns a scaled rotation value that can be used with drivetrain.turn_to_rotation()
    @staticmethod
    def calc_rotation_at_heading(heading):
        # First get the robot's total "real" rotation and heading - be careful not to read the inertial sensor twice in the same routine
        # in case it gets updated.
        current_rotation = GyroHelper.gyro_rotation()
        current_heading = GyroHelper.to_heading(current_rotation)

        # Calculate the real heading and angle delta to get to the desired heading
        delta_heading = heading - current_heading
        delta_angle = GyroHelper.to_angle(delta_heading)

        # The new rotation value will be the current + the angle delta * scale factor
        new_rotation = current_rotation + delta_angle
        new_rotation *= GYRO_SCALE_FOR_TURNS

        # Return value can be used with drivetrain.turn_to_rotation() - will not work with drivetrain.turn_to_heading()
        return new_rotation

class Tracking:
    global inertial, left_drive, right_drive

    class Orientation:
        def __init__(self, x, y, rotation):
            self.x = x
            self.y = y
            self.rotation = rotation

    # Tracking wheel geometry
    # In this case we are just uising morot encoders and gyro, however same concept works for odometry wheels
    GEAR_RATIO = 24.0 / 60.0 # external gear ratio
    WHEEL_SIZE = 0.260 # m
    # FWD_OFFSET is the distance from the robot center to the forward tracking wheel, right is positive
    FWD_OFFSET = 0.0 # m
    # SIDE_OFFSET is the distance from the robot center to the side tracking wheel, forward is positive
    SIDE_OFFSET = 0.0 # m

    this_instance = None

    def __init__(self, x, y, heading, initial_left_position, initial_right_position, initial_theta):
        self.x = x # meters NORTH
        self.y = y # meters EAST
        # heading passed in as degrees 0 to 360. Converted to continuous radians
        # true_theta is our internal heading in radians
        # This is a true direction so no gyro scale applied here
        self.true_theta = self.to_angle(radians(heading)) # continuous radians
        # theta is used to compare gyro readings. Its separate from actual direction (true_theta) of the robot so we can override direction
        # without resetting the gyro readings
        # Captures the starting value which typically is +/- 1 degrees after calibration
        # Gyro scale is not strictly needed here but seeing as we are dealing with small values it won't impact things if its
        # applied or not
        self.theta = initial_theta # radians
        self.timestep = 0.01 # seconds

        self.previous_left_position = initial_left_position # revolutions
        self.previous_right_position = initial_right_position # revolutions
        self.previous_theta = initial_theta # radians

    def set_orientation(self, x, y, heading):
        self.x = x
        self.y = y
        self.theta = self.to_angle(radians(heading)) * GYRO_SCALE_FOR_READOUT # continuous radians

    # helper function for converting angles
    # mimics inertial.angle() producing result in range (-180, 180])
    # note: degrees in and out
    def to_angle(self, rotation):
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle

    # mimics inertial.heading() producing result in range [0, 360)
    # note: degrees in and out
    def to_heading(self, rotation):
        return rotation % 360.0
    
    # returns internal theta (radians) to degrees heading [0, 360)
    def current_heading(self):
        heading_deg = degrees(self.true_theta)
        return self.to_heading(heading_deg)

    def calc_timestep_arc_chord(self, x, y, theta, delta_forward, delta_side, delta_theta):
        # x, y, delta_forward, delta_side in meters
        # theta, delta_theta in radians

        # local deltas
        if (delta_theta == 0.0):
            # no turn - use simple deltas
            delta_local_x = delta_forward
            delta_local_y = delta_side
            to_global_rotation_angle = theta
        else:
            # robot turning
            # calculate radius of movement for forward and side wheels
            r_linear = Tracking.FWD_OFFSET + (delta_forward / delta_theta) # m
            r_strafe = Tracking.SIDE_OFFSET + (delta_side / delta_theta) # m

            # calculate chord distances using chord length = 2 * r * sin(theta / 2)
            # pre-rotate by half the turn angle so we have only distance along one axis for each
            # when we rotate to global frame we need to account for this half-angle rotation
            to_global_rotation_angle = theta + delta_theta / 2
            delta_local_x = r_linear * 2.0 * sin(delta_theta / 2.0)
            delta_local_y = r_strafe * 2.0 * sin(delta_theta / 2.0)

        # rotate to global
        delta_global_x = delta_local_x * cos(to_global_rotation_angle) - delta_local_y * sin(to_global_rotation_angle)
        delta_global_y = delta_local_x * sin(to_global_rotation_angle) + delta_local_y * cos(to_global_rotation_angle)

        return (x + delta_global_x, y + delta_global_y, theta + delta_theta)

    def update_location(self, left_position, right_position, theta):
        left_position *= Tracking.GEAR_RATIO
        right_position *= Tracking.GEAR_RATIO

        delta_left = left_position - self.previous_left_position
        delta_right = right_position - self.previous_right_position
        delta_theta = theta - self.previous_theta

        delta_forward = Tracking.WHEEL_SIZE * (delta_left + delta_right) / 2.0
        delta_side = 0.0 # no side encoder

        self.x, self.y, self.theta = self.calc_timestep_arc_chord(self.x, self.y, self.theta, delta_forward, delta_side, delta_theta)
        self.true_theta += delta_theta

        self.previous_left_position = left_position
        self.previous_right_position = right_position
        self.previous_theta = theta
    
    def get_orientation(self):
        return Tracking.Orientation(self.x, self.y, degrees(self.theta))
    
    @staticmethod
    def gyro_theta(sensor):
        return radians(Tracking.gyro_rotation(sensor))

    @staticmethod
    def gyro_rotation(sensor):
        return sensor.rotation() * GYRO_SCALE_FOR_READOUT

    @staticmethod
    def get_instance():
        return Tracking.this_instance

    @staticmethod
    def tracker_thread():
        # print(args)
        tracker = Tracking(0, 0, 0, left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), Tracking.gyro_theta(inertial))
        Tracking.this_instance = tracker
        loop_count = 0
        while(True):
            tracker.update_location(left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), Tracking.gyro_theta(inertial))
            # if (loop_count % 100 == 0):
            #     print("X: {:.2f} m, Y: {:.2f} m, Heading: {:.2f} deg".format(tracker.x, tracker.y, tracker.current_heading()))
            loop_count += 1
            wait(tracker.timestep, SECONDS)


# "Simple" PID controller class for demonstration purposes only
# This provides the basic functionality required by most controllers including feedforward
# The output range should be in the range [-1.0, 1.0]
# Input scaling is not performed, so to not have very large or small K values pick an input range that makes sense, e.g. degrees
# works well for turning with a Kp of around 1.0. Similarly using wheel revolutions in degrees for tracking distance does the same
# thing with a drive Kp of 1.0 being a good starting point
# K values are also scaled by the timestep to help get Kp values roughly in the ballpark
# Some required features implemnted include:
# - Integral wind-up protection using backcalculation of the saturation limit of the controller, and also zero crossing detection
#   Note zero crossing is not generally recommended, but it helps make the code simpler
# - Programmable settling time, threshold and timeout values. Setting these to 0.0 will let controller run indefinitely (e.g. for heading lock)
# - Output and ramp limits allow for controlling max output swing as well as acceleration
# Not implemented:
# - Proper initialization of last values in the case that the inputs are not zero at the start
# - Resetting state. Its assumed the class is created for each separate command needing PID
# - Deadband control. In the case where the robot needs a minimum power to get it to move, outputs below this obviously won't achieve anything
class SimplePID:

    def __init__(self, Kp, Ki, Kd, Kf = 0.0):
        self.timestep = 0.01 # approximate timestep in seconds - used to process timeouts. Changing this will scale the K values
        
        # Constants
        self.Kp = Kp * self.timestep
        self.Ki = Ki * self.timestep
        self.Kd = Kd * self.timestep
        self.Kf = Kf

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

        # Time parameters
        self.settle_timer_limit = 0.1 # default settle time in seconds, set to 0.0 to disable settling
        self.settle_error_threshold = 1.0 # unit agnostic settle threshold
        self.timeout_timer_limit = 10.0 # default timeout in seconds, set to 0.0 to disable timeout

        # Timers
        # For free running PID such as heading lock disable these by setting time limits above to 0.0
        self.settle_timer = self.settle_timer_limit
        self.timeout_timer = self.timeout_timer_limit

        # Output limits and integral windup controls
        self.output_limit = 1.0 # default output limit (actual output will be clamped to -output_limit, +output_limit)
        self.output_ramp_limit = 0.0 # default output ramp limit (max change in output per compute() call). 0.0 = no ramp limit
        self.integral_limit = self.output_limit / self.Kp # default integral limit (integral term will be clamped to -integral_limit, +integral_limit)

        # Output flags
        self.is_timed_out = False
        self.is_settled = False

    # Setter functions
    def set_settle_time(self, time_sec):
        self.settle_timer_limit = time_sec

    def set_timeout(self, time_sec):
        self.timeout_timer_limit = time_sec

    # Our settle threshold will be in our measurement units. If degrees, we want this to be about 0.5degrees or less
    def set_settle_threshold(self, threshold):
        self.settle_error_threshold = threshold

    def set_timestep(self, timestep_sec):
        self.Ki *= timestep_sec / self.timestep
        self.Kd /= timestep_sec / self.timestep
        self.Kp /= timestep_sec / self.timestep
        self.timestep = timestep_sec

    # Output should be normalized to range [-1.0, +1.0]
    def set_output_limit(self, limit):
        self.output_limit = limit
        self.set_integral_limit(self.output_limit / self.Kp)

    # Ramp limit will be based on our normaled output range and timestep
    # So for a timestep of 0.01sec and a ramp limit of 0.1, the output can change by a maximum of 0.1 every 0.01sec
    # If output_limit is set to 1.0 this means it will take at least 0.2sec to go from -1.0 to +1.0 output
    def set_output_ramp_limit(self, ramp_limit):
        self.output_ramp_limit = ramp_limit

    # Integral limit will be based on our units of measurement and take the output_limit into consideration
    # E.g. for a heading controller we will want to limit the output to the range [-output_limit, +output_limit] which will
    # represent how fast we want each side to turn in the default range of [-1.0, +1.0] (1.0 = full power or 100%)
    # If the heading error is large enough we just want the maximum output and do not want the integral term to accumlate
    # The point at which we fall under the output limit will be when we transition from the saturation region to the controlable
    # region. This will typically be around an error value of around 10 degrees but will depend on the Kp value needed, so we
    # can only set this once we have some idea of what Kp will be.
    # Ideally our resulting error should represent some physically meaningful value so error in this case would be in degrees
    # E.g. if we set output_limit to 0.5 (50% power) and Kp to 0.01 we would saturate the output down to an error of 50deg
    #  saturation point = output_limit = Kp * error  => error = output_limit / Kp or 0.5 % / 0.01 Kp = 50 degrees
    # Therefore our integral limit would be set to 50 (degrees) in this case
    def set_integral_limit(self, limit):
        self.integral_limit = limit

    # Getter functions
    def get_is_timed_out(self):
        return self.is_timed_out
    
    def get_is_settled(self):
        return self.is_settled

    # Main compute function
    def compute(self, setpoint, measurement):
        if self.is_done(): return 0.0

        error = setpoint - measurement

        # Integral windup control
        # Case 1: only accumulate integral if error is less than saturation limit. We reset to zero to allow for changing setpoints
        if abs(error) < self.integral_limit:
            self.integral += error
        else:
            self.integral = 0.0
        # Case 2: reset integral if error crosses zero
        if (error > 0.0 and self.prev_error < 0.0) or (error < 0.0 and self.prev_error > 0.0):
            self.integral = 0.0

        derivative = error - self.prev_error

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative) + (self.Kf * setpoint)

        self.prev_error = error

        # Output limiting
        is_output_limited, output = self.limit(output, self.output_limit)
        
        # Output ramp limiting
        is_ramp_limited = False
        if (self.output_ramp_limit > 0.0):
            is_ramp_limited, output = self.ramp_limit(output, self.prev_output, self.output_ramp_limit)

        self.prev_output = output

        # TODO: minimum output for small errors

        # Timeouts and settling
        if abs(error) < self.settle_error_threshold and not is_ramp_limited:
            self.settle_timer -= self.timestep
        else:
            self.settle_time = self.settle_timer_limit
        self.timeout_timer -= self.timestep

        if (self.is_done()): return 0.0

        # print(error, output)

        return output
    
    # Timeout and settle check function
    def is_done(self):
        if (self.settle_timer_limit > 0.0 and self.settle_error_threshold > 0.0 and self.settle_timer <= 0.0):
            self.is_settled = True
            return True
        
        if self.timeout_timer_limit > 0.0 and self.timeout_timer <= 0.0:
            self.is_timed_out = True
            return True
        
        return False
    
    # Output limiting
    def limit(self, value, limit):
        limited_output = value
        is_limited = False
        if value > limit:
            limited_output = limit
        elif value < -limit:
            limited_output = -limit
        if (limited_output != value): is_limited = True

        return is_limited, limited_output
    
    def ramp_limit(self, value, prev_value, limit):
        ramp_limited_output = value
        is_ramp_limited = False
        if (abs(value - prev_value) > limit):
            if (value > prev_value): ramp_limited_output = prev_value + limit
            else: ramp_limited_output = prev_value - limit
        if (value != ramp_limited_output): is_ramp_limited = True

        return is_ramp_limited, ramp_limited_output
    
# Simple drivetrain proxy. This replaces the VEX provided DriveTrain or SmartDrive classes with one that uses the inertial
# sensor for turning and optionally for driving straight. It uses the SimplePID class
# There are 3 sets of PID parameters needed to make this work
# - Turning: see set_turn_*()
# - Driving: see set_drive_*()
# - Heading lock while driving: see set_heading_lock_*(). Note this is different from pure turns and runs at the same time as driving
# All functiionality is implemented in the turn_for() and drive_for() functions
class SimpleDrive:

    MAX_VOLTAGE = 11.5

    class PIDParameters:
        def __init__(self):
            self.Kp = 1.0
            self.Ki = 0.0
            self.Kd = 0.0
            self.max_output = 1.0
            self.max_ramp = 1.0
            self.settle_error = 1.0

    def __init__(self, left_motors, right_motors, wheel_travel_mm=320.0, ext_gear_ratio=1.0):
        self.turn_pid_constants = SimpleDrive.PIDParameters()
        self.drive_pid_constants = SimpleDrive.PIDParameters()
        self.heading_lock_pid_constants = SimpleDrive.PIDParameters()

        self.left_motors = left_motors
        self.right_motors = right_motors
        self.wheel_travel_mm = wheel_travel_mm
        self.ext_gear_ratio = ext_gear_ratio

        self.stop_mode = BrakeType.COAST

    def set_drive_velocity(self, velocity, unit):
        self.drive_pid_constants.max_output = velocity / 100.0

    def set_drive_acceleration(self, acceleration, unit):
        self.drive_pid_constants.max_ramp = acceleration / 100.0

    def set_drive_constants(self, Kp, Ki, Kd, settle_error):
        self.drive_pid_constants.Kp = Kp
        self.drive_pid_constants.Ki = Ki
        self.drive_pid_constants.Kd = Kd
        # settle error will be in MM, we need to convert to degree revolutions for internal use
        self.drive_pid_constants.settle_error = 360.0 * settle_error / (self.wheel_travel_mm * self.ext_gear_ratio)

    def set_turn_velocity(self, velocity, unit):
        self.turn_pid_constants.max_output = velocity / 100.0
        self.heading_lock_pid_constants.max_output = velocity / 100.0

    def set_turn_acceleration(self, acceleration, unit):
        self.turn_pid_constants.max_ramp = acceleration / 100.0

    def set_turn_constants(self, Kp, Ki, Kd, settle_error):
        self.turn_pid_constants.Kp = Kp
        self.turn_pid_constants.Ki = Ki
        self.turn_pid_constants.Kd = Kd
        self.turn_pid_constants.settle_error = settle_error

    def set_heading_lock_constants(self, Kp, Ki, Kd, settle_error):
        self.heading_lock_pid_constants.Kp = Kp
        self.heading_lock_pid_constants.Ki = Ki
        self.heading_lock_pid_constants.Kd = Kd
        self.heading_lock_pid_constants.settle_error = settle_error

    def set_timeout(self, time, unit):
        # placeholder for setting timeout if needed
        pass

    def set_stopping(self, mode):
        self.stop_mode = mode

    def turn_to_heading(self, heading):
        angle = GyroHelper.calc_angle_to_heading(heading)
        self.turn_for(RIGHT, angle, DEGREES)

    def turn_for(self, direction, angle, unit):
        turn_pid = SimplePID(self.turn_pid_constants.Kp, self.turn_pid_constants.Ki, self.turn_pid_constants.Kd)
        turn_pid.set_output_limit(self.turn_pid_constants.max_output) # limit output to 50% power
        turn_pid.set_settle_threshold(self.turn_pid_constants.settle_error) # settle threshold in degrees
        start_rotation = GyroHelper.gyro_rotation()
        target_rotation = start_rotation + (angle if direction == TurnType.RIGHT else -angle)
        while not turn_pid.is_done():
            current_rotation = GyroHelper.gyro_rotation()
            pid_output = turn_pid.compute(target_rotation, current_rotation)

            drive_voltage = pid_output * SimpleDrive.MAX_VOLTAGE # scale to voltage
            if (drive_voltage > SimpleDrive.MAX_VOLTAGE): drive_voltage = SimpleDrive.MAX_VOLTAGE
            if (drive_voltage < -SimpleDrive.MAX_VOLTAGE): drive_voltage = -SimpleDrive.MAX_VOLTAGE

            self.left_motors.spin(FORWARD, drive_voltage, VOLT)
            self.right_motors.spin(FORWARD, -drive_voltage, VOLT)
            wait(turn_pid.timestep, SECONDS)

        self.stop(self.stop_mode)
        print("Done Turn: ", turn_pid.get_is_settled(), turn_pid.get_is_timed_out())

    def drive_for(self, direction, distance, unit, heading = None):
        drive_pid = SimplePID(self.drive_pid_constants.Kp, self.drive_pid_constants.Ki, self.drive_pid_constants.Kd)
        drive_pid.set_output_limit(self.drive_pid_constants.max_output) # limit output to 50% power
        drive_pid.set_settle_threshold(self.drive_pid_constants.settle_error) # settle threshold in degrees

        if (heading is not None):
            turn_pid = SimplePID(self.heading_lock_pid_constants.Kp, self.heading_lock_pid_constants.Ki, self.heading_lock_pid_constants.Kd)
            turn_pid.set_output_limit(self.heading_lock_pid_constants.max_output) # limit output to 50% power
            turn_pid.set_settle_time(0.0)
            turn_pid.set_timeout(0.0)

            target_rotation = GyroHelper.calc_rotation_at_heading(heading)

        left_start_pos = self.left_motors.position(RotationUnits.DEG)
        right_start_pos = self.right_motors.position(RotationUnits.DEG)

        target_distance_revs = 360.0 * distance / (self.wheel_travel_mm * self.ext_gear_ratio) # convert mm to wheel revolutions assuming 100mm diameter wheels
        target_position = target_distance_revs if direction == DirectionType.FORWARD else -target_distance_revs

        while not drive_pid.is_done():
            average_position = (
                (self.left_motors.position(RotationUnits.DEG) - left_start_pos) +
                (self.right_motors.position(RotationUnits.DEG) - right_start_pos)) / 2.0

            pid_output = drive_pid.compute(target_position, average_position)

            turn_pid_output = 0.0
            if (heading is not None):
                current_rotation = GyroHelper.gyro_rotation()
                turn_pid_output = turn_pid.compute(target_rotation, current_rotation)

            left_output = pid_output + turn_pid_output
            right_output = pid_output - turn_pid_output

            left_voltage = left_output * SimpleDrive.MAX_VOLTAGE # scale to voltage
            if (left_voltage > SimpleDrive.MAX_VOLTAGE): left_voltage = SimpleDrive.MAX_VOLTAGE
            if (left_voltage < -SimpleDrive.MAX_VOLTAGE): left_voltage = -SimpleDrive.MAX_VOLTAGE

            right_voltage = right_output * SimpleDrive.MAX_VOLTAGE # scale to voltage
            if (right_voltage > SimpleDrive.MAX_VOLTAGE): right_voltage = SimpleDrive.MAX_VOLTAGE
            if (right_voltage < -SimpleDrive.MAX_VOLTAGE): right_voltage = -SimpleDrive.MAX_VOLTAGE

            self.left_motors.spin(FORWARD, left_voltage, VOLT)
            self.right_motors.spin(FORWARD, right_voltage, VOLT)
            wait(drive_pid.timestep, SECONDS)

        self.stop(self.stop_mode)
        print("Done Drive: ", drive_pid.get_is_settled(), drive_pid.get_is_timed_out())

    def stop(self, mode):
        # Note that setting mode to None will keep motors at their last commanded output
        if (mode is not None):
            self.left_motors.stop(mode)
            self.right_motors.stop(mode)

# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def demo1_drive_straight(drive_train):
    drive_train.turn_to_heading(90.0)
    drive_train.turn_to_heading(0.0)
    drive_train.drive_for(FORWARD, 36 * 25.4, MM, 0.0)
    drive_train.drive_for(REVERSE, 36 * 25.4, MM, 0.0)
    drive_train.turn_to_heading(0.0)

# DEMO2: Once robot has been tuned for a full turn, use this to test turning to specific headings
def demo2_turn_to_headings(drive_train):
    headings = [0, 90, 180, 270, 0, 90, 180, 270, 0, 270, 180, 90, 0, 270, 180, 90, 0]
    for heading in headings:
        brain.screen.print("Turning to Heading: ", heading)
        brain.screen.next_row()
        print("Turning to Heading: ", heading)
        # choose either version of turn_to_heading() here
        # turn_to_heading1(heading)
        drive_train.turn_to_heading(heading)
        current_heading = GyroHelper.gyro_heading()
        brain.screen.print("Current Heading: ", current_heading)
        brain.screen.next_row()
        print("Current Heading: ", current_heading)
        wait(1, SECONDS)
        
def autonomous():
    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    # place automonous code here
    pass

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("Waiting for robot to initialize fully ... ")
    brain.screen.next_row()

    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    brain.screen.print("done")
    brain.screen.next_row()

    # Good idea to check if inertial sensor is present before using it as unexpected motion can occur
    if not inertial.installed():
        brain.screen.print("NO INERTAIL SENSOR")
        while True:
            wait(20, MSEC)

    # place user control code here
    drive_train = SimpleDrive(left_drive, right_drive)
    drive_train.set_turn_constants(Kp=1.0, Ki=0.04, Kd=10.0, settle_error=0.5) # degrees
    drive_train.set_drive_constants(Kp=1.0, Ki=0.0, Kd=0.0, settle_error=5) # mm
    drive_train.set_heading_lock_constants(Kp=1.0, Ki=0.0, Kd=0.0, settle_error=0.0) # degrees
    drive_train.set_turn_velocity(50, PERCENT)
    drive_train.set_drive_velocity(50, PERCENT)

    my_tracker = Tracking.get_instance()

    # demo1_drive_straight(drive_train)
    # demo2_turn_to_headings(drive_train)

    # place driver control in this while loop
    while True:
        if (my_tracker is not None):
            print("X: {:.2f} m, Y: {:.2f} m, Heading: {:.2f} deg".format(my_tracker.x, my_tracker.y, my_tracker.current_heading()))
        wait(1, SECONDS)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
