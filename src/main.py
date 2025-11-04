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
from math import sin, cos, radians, degrees, atan2
from collections import namedtuple

brain = Brain()

# DEVICE DECLARATIONS

# declare motors
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)
MOTOR_SPEED_RPM = 200

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


# Tracking class to calculate robot position using tracking wheels or motor encoders plus inertial sensor
# Internally everything is based in meters and radians
# To distinguish from VEX uses for various heading readings, we use the following terminology:
# - ROTATION: continuous rotation value in degrees (can be positive or negative, no bounds)
# - HEADING: bounded heading in degrees [0, 360)
# - ANGLE: bounded angle in degrees (-180, +180]
# - THETA: continuous rotation value in radians (can be positive or negative, no bounds). Same as ROTATION but in radians
# THETA is used internally and converted to/from HEADING/ANGLE as needed
# Note that  __init__() and update_location() assume that the gyro scale factor has already been applied to the inertial sensor readings
class Tracking:
    global inertial, left_drive, right_drive

    Orientation = namedtuple('Orientation', ['x', 'y', 'heading'])
    
    # Configuration initializers
    # @param *_wheel_size is the the size of the odometry wheel. Set to 0.0 if not present. If only one forward wheels is present, use left regardless
    #  if it is mounted on the left of the right of the robot
    # @param *_gear_ratio is any gear ratio used
    # @param *_offset is the offset of the tracking wheel relative to the turning center of the robot. Positive left/right is to RIGHT of robot, so
    #  wheels mounted on left of robot would be negative. Positive side is to FRONT of robot, so wheels mounted towards the back of the robot
    #  would be negative
    # @param fwd_is_odom. If False left and rigght wheels are trated as the motor left and right motor group encoders
    # It is assumed that encoders are configured correctly such that FORWARD motion is positive for both left and right encoders and RIGHT motiion
    # is positive to side/strafe encoder

    Configuration = namedtuple('Configuration', [
            'left_wheel_size', 'left_gear_ratio', "left_offset",
            'right_wheel_size', 'right_gear_ratio', 'right_offset',
            'fwd_is_odom',
            'side_wheel_size', 'side_gear_ratio', 'side_offset'
       ])
    
    # Encoder initializers
    # @param left is initial left encoder position in revolutions (either left motors or left odom wheek)
    # @param right is initial right encoder position in revolutions (either right motors or right odom wheel)
    # @param side is sideways or strafe encoder in revolutions (only valid if sideways odom wheel installed)
    # @param theta is initial gyro theta in radians (currently not used - the inertial sensor will be programmed to the initial heading)
    # This assumes that when declaring devices that robot forward is positive and robot right is positive. Positive rotation is towards the right
    EncoderValues = namedtuple('EncoderValues', ['left', 'right', 'side', 'theta'])

    # Tracking wheel geometry
    # In this case we are just using motor encoders and gyro, however same concept works for odometry wheels
    DEFAULT_GEAR_RATIO = 60.0 / 60.0 # external gear ratio
    DEFAULT_WHEEL_SIZE = 320.0 # mm
    # FWD_OFFSET is the distance from the robot center to the forward tracking wheel, right is positive
    DEFAULT_FWD_OFFSET = 0.0 # mm
    # SIDE_OFFSET is the distance from the robot center to the side tracking wheel, forward is positive
    DEFAULT_SIDE_OFFSET = 0.0 # mm

    # this_instance will hold the singleton instance of the tracker. This is a way to have all the tracking code and the associated thread
    # as part of the same class to keep all the code together
    # The tracker_thread() is started from  pre_autonomous(), or somewhere early in the program and sets this_instance once initialized
    THIS_INSTANCE = None
    INITIALIZED = False

    # Initializer
    # @param x is initial NORTH position in MM
    # @param y is initial EAST position in MM
    # @param heading is initial true heading of robot in degrees [0, 360). 0 deg is NORTH
    # @param configuration (optional) is the configuration of the wheels used for odometry
    # @param inital_values (optional) is the initial values of the encoders used if not zero. Not that theta (heading) will be ignored at the moment
    def __new__(cls, *args, **kwargs):
        print('new')
        if cls.THIS_INSTANCE is None:
            cls.THIS_INSTANCE = super().__new__(cls)
        return cls.THIS_INSTANCE

    def __init__(self, orientation: Union[Orientation, None] = None, configuration: Union[Configuration, None] = None, initial_values: Union[EncoderValues, None] = None):
        print('init')

        if (self.INITIALIZED): return
        self.INITIALIZED = True

        x = 0.0 if orientation is None else orientation.x
        y = 0.0 if orientation is None else orientation.y
        heading = 0.0 if orientation is None else orientation.heading

        self.x = x # MM NORTH
        self.y = y # MM EAST

        # heading passed in as degrees 0 to 360. Converted to continuous radians
        # theta is our internal (continous) rotation in radians 
        # theta reflects the true rotation of the robot not the uncorrected gyro version
        # We also set reset the gyro reading to match our heading. The set_sensor_heading() call will apply the gyro scaling factor
        self.theta = self.to_angle(radians(heading))
        self.set_sensor_heading(heading)

        # Configuration
        self.fwd_is_odom = False
        self.left_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.left_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.left_offset = Tracking.DEFAULT_FWD_OFFSET

        self.right_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.right_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.right_offset = Tracking.DEFAULT_FWD_OFFSET

        self.side_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.side_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.side_offset = Tracking.DEFAULT_SIDE_OFFSET
        if (configuration is not None): self.set_configuration(configuration)

        self.timestep = 0.01 # seconds

        # Capture initial values of encoders and store as the previous values
        self.previous_left_position = 0.0 if initial_values is None else initial_values.left # revolutions
        self.previous_right_position = 0.0 if initial_values is None else initial_values.right # revolutions
        self.previous_side_position =  0.0 if initial_values is None else initial_values.side # revolutions
        self.previous_theta = self.theta # radians

    def set_configuration(self, configuration: Configuration):
        self.fwd_is_odom = configuration.fwd_is_odom
        self.left_wheel_size = configuration.left_wheel_size
        self.left_gear_ratio = configuration.left_gear_ratio
        self.left_offset = configuration.left_offset

        self.right_gear_ratio = configuration.right_gear_ratio
        self.right_wheel_size = configuration.right_wheel_size
        self.right_offset = configuration.right_offset

        self.side_wheel_size = configuration.side_wheel_size
        self.side_gear_ratio = configuration.side_gear_ratio
        self.side_offset = configuration.side_offset
   
    # returns internal theta (radians) in degrees heading [0, 360)
    # theoretically this is same as calling GyroHelper.gyro_heading()
    def current_heading(self):
        heading_deg = degrees(self.theta)
        return self.to_heading(heading_deg)

    def calc_timestep_arc_chord(self, x, y, theta, delta_forward, delta_side, delta_theta):
        # x, y, delta_forward, delta_side in MM
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
            r_linear = self.left_offset + (delta_forward / delta_theta) # mm
            r_strafe = self.side_offset + (delta_side / delta_theta) # mm

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

    def update_location(self, left_position, right_position, side_position, theta):
        # position here is the rotoation of th wheel so needs to be multiplied by any gear ratio if present
        left_position *= self.left_gear_ratio
        right_position *= self.right_gear_ratio
        side_position *= self.side_gear_ratio

        delta_left = left_position - self.previous_left_position
        delta_right = right_position - self.previous_right_position
        delta_side = side_position - self.previous_side_position
        delta_theta = theta - self.previous_theta

        # delta_forward and delta_strafe will be the piecewise motion of this robot in this timestop, for forward and sideways/strafe in mm
        if (self.right_wheel_size <= 0.0):
            delta_forward = self.left_wheel_size * delta_left
        else:
            delta_forward = self.left_wheel_size * (delta_left + delta_right) / 2.0
        delta_strafe = self.side_wheel_size * delta_side

        self.x, self.y, self.theta = self.calc_timestep_arc_chord(self.x, self.y, self.theta, delta_forward, delta_strafe, delta_theta)

        self.previous_left_position = left_position
        self.previous_right_position = right_position
        self.previous_side_position = side_position
        self.previous_theta = theta
    
    def get_orientation(self):
        return Tracking.Orientation(self.x, self.y, self.to_heading(degrees(self.theta)))

    def set_orientation(self, orientation: Orientation):
        self.x = orientation.x
        self.y = orientation.y
        self.theta = radians(self.to_angle(orientation.heading))
        self.previous_theta = self.theta
        self.set_sensor_heading(orientation.heading)

    def trajectory_to_point(self, x, y):
        distance = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
        heading = self.to_heading(degrees(atan2(y - self.y, x - self.x)))
        return distance, heading
    
    # helper function for converting angles
    # mimics inertial.angle() producing result in range (-180, 180])
    # note: degrees in and out
    @staticmethod
    def to_angle(rotation):
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle

    # mimics inertial.heading() producing result in range [0, 360)
    # note: degrees in and out
    @staticmethod
    def to_heading(rotation):
        return rotation % 360.0
    
    @staticmethod
    def gyro_theta(sensor):
        return radians(Tracking.gyro_rotation(sensor))

    @staticmethod
    def gyro_rotation(sensor: Inertial):
        return sensor.rotation() * GYRO_SCALE_FOR_READOUT

    #@staticmethod
    #def get_instance():
    #    return Tracking.THIS_INSTANCE
    
    @staticmethod
    def set_sensor_heading(heading):
        angle = Tracking.to_angle(heading)
        rotation = angle * GYRO_SCALE_FOR_TURNS
        inertial.set_rotation(rotation, DEGREES)

    @staticmethod
    def tracker_thread():
        # print(args)
        initial_encoders = Tracking.EncoderValues(left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), 0.0, Tracking.gyro_theta(inertial)) 
        tracker = Tracking(None, None, initial_encoders)
        Tracking.THIS_INSTANCE = tracker
        while(True):
            tracker.update_location(left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), 0.0, Tracking.gyro_theta(inertial))
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

        # self.log = []

    # Setter functions
    def set_settle_time(self, time_sec):
        self.settle_timer_limit = time_sec

    def set_timeout(self, time_sec):
        self.timeout_timer_limit = time_sec
        self.timeout_timer = time_sec

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

        derivative = error - self.prev_error

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative) + (self.Kf * setpoint)

        # Output limiting
        is_output_limited, output = self.limit(output, self.output_limit)
        
        # Output ramp limiting
        is_ramp_limited = False
        if (self.output_ramp_limit > 0.0):
            is_ramp_limited, output = self.ramp_limit(output, self.prev_output, self.output_ramp_limit)

        self.prev_output = output

        # Integral windup control
        # Case 1: only accumulate integral if error is less than saturation limit. We reset to zero to allow for changing setpoints
        if abs(error) > self.integral_limit:
            self.integral = error
        # Case 2: reset integral if error crosses zero
        elif (error > 0.0 and self.prev_error < 0.0) or (error < 0.0 and self.prev_error > 0.0):
            self.integral = 0.0
        elif (is_output_limited or is_ramp_limited):
            self.integral = error
        else:
            self.integral += error

        self.prev_error = error

        # TODO: minimum output for small errors

        # Timeouts and settling
        if abs(error) < self.settle_error_threshold and not is_ramp_limited:
            self.settle_timer -= self.timestep
        else:
            self.settle_time = self.settle_timer_limit
        self.timeout_timer -= self.timestep

        if (self.is_done()): return 0.0

        # self.log.append([error, output, self.integral])

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
            # if (value > 0.0 and value > prev_value): ramp_limited_output = prev_value + limit
            # elif (value < 0.0 and value < prev_value): ramp_limited_output = prev_value - limit
            if (value > prev_value): ramp_limited_output = prev_value + limit
            elif (value < prev_value): ramp_limited_output = prev_value - limit
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
    MAX_PERCENT = 50.0 # for same K values tuned for voltage, need to slow percent motor control down
    USE_VOLTAGE = True

    class PIDParameters:
        def __init__(self):
            self.Kp = 1.0
            self.Ki = 0.0
            self.Kd = 0.0
            self.max_output = 1.0
            self.max_ramp = 1.0
            self.settle_error = 1.0

    def __init__(self, left_motors: MotorGroup, right_motors: MotorGroup, motor_speed=MOTOR_SPEED_RPM, wheel_travel_mm=320.0, ext_gear_ratio=1.0):
        self.turn_pid_constants = SimpleDrive.PIDParameters()
        self.drive_pid_constants = SimpleDrive.PIDParameters()
        self.heading_lock_pid_constants = SimpleDrive.PIDParameters()
        self.default_timeout = 10.0 # seconds

        self.left_motors = left_motors
        self.right_motors = right_motors
        self.motor_speed = motor_speed
        self.wheel_travel_mm = wheel_travel_mm
        self.ext_gear_ratio = ext_gear_ratio

        self.stop_mode = BrakeType.COAST

        self.drive_velocity = 100.0 # percent

    def set_drive_velocity(self, velocity, unit):
        self.drive_velocity = velocity
        self.drive_pid_constants.max_output = velocity / 100.0

    def set_drive_acceleration(self, acceleration, unit):
        self.drive_pid_constants.max_ramp = acceleration / 100.0

    # settle error will be in MM, we need to convert to degree revolutions for internal use
    def calc_drive_settle_error(self, settle_error):
        return 360.0 * settle_error / (self.wheel_travel_mm * self.ext_gear_ratio)

    def set_drive_constants(self, Kp, Ki, Kd, settle_error):
        self.drive_pid_constants.Kp = Kp
        self.drive_pid_constants.Ki = Ki
        self.drive_pid_constants.Kd = Kd
        self.drive_pid_constants.settle_error = self.calc_drive_settle_error(settle_error)

    def set_turn_velocity(self, velocity, unit):
        self.turn_pid_constants.max_output = velocity / 100.0
        self.heading_lock_pid_constants.max_output = velocity / 100.0

    def set_turn_acceleration(self, acceleration, unit):
        self.turn_pid_constants.max_ramp = acceleration / 100.0

    def set_turn_constants(self, Kp, Ki, Kd, settle_error):
        self.turn_pid_constants.Kp = Kp
        self.turn_pid_constants.Ki = Ki
        self.turn_pid_constants.Kd = Kd
        # degrees
        self.turn_pid_constants.settle_error = settle_error

    def set_heading_lock_constants(self, Kp, Ki, Kd, settle_error):
        self.heading_lock_pid_constants.Kp = Kp
        self.heading_lock_pid_constants.Ki = Ki
        self.heading_lock_pid_constants.Kd = Kd
        self.heading_lock_pid_constants.settle_error = settle_error

    def set_timeout(self, time):
        self.default_timeout = time

    def set_stopping(self, mode):
        self.stop_mode = mode

    # Returns approx max linear speed of robot (m/s) - useful for timeout calculations
    # If using voltage for motors, top speed will be somewhat above stated RPM of cartridge
    # Will not take into account acceleration, deceleration and settle time - pad appropriately
    def linear_speed(self):
        # Motor speed in RPM and wheel size in MM
        print(self.motor_speed, self.ext_gear_ratio, self.wheel_travel_mm, self.drive_velocity)
        return self.motor_speed * self.ext_gear_ratio * self.wheel_travel_mm * self.drive_velocity / (1000.0 * 60.0 * 100.0)
    
    # Approximate max rotational velocity of robot in deg/s
    # Using most common builds having mix of traction and omni wheels a 200RPM drive on 4" wheels should be
    # able to turn 360deg in 1 sec, so we scale this heuristic appropriately
    def rotation_speed(self):
        reference_linear_speed = 1.067 # m/s - speed of 200RPM drive using 4" wheels and 1:1 gear ratio
        reference_rotation_speed = 360.0 # deg/s
        return reference_rotation_speed * self.linear_speed() / reference_linear_speed

    def turn_to_heading(self, heading, settle_error = None, timeout = None):
        angle = GyroHelper.calc_angle_to_heading(heading)
        self.turn_for(RIGHT, angle, DEGREES, settle_error=settle_error, timeout=timeout)

    def turn_for(self, direction, angle, unit, settle_error = None, timeout = None):
        turn_pid = SimplePID(self.turn_pid_constants.Kp, self.turn_pid_constants.Ki, self.turn_pid_constants.Kd)
        turn_pid.set_output_limit(self.turn_pid_constants.max_output) # limit output to defined power
        turn_pid.set_output_ramp_limit(self.turn_pid_constants.max_ramp)
        # allow for per call settle_threshold and timeout, useful if we need to vary accuracy particularly when chaining motions
        turn_pid.set_settle_threshold(self.turn_pid_constants.settle_error if settle_error is None else settle_error) # settle threshold in degrees
        turn_pid.set_timeout(self.default_timeout if timeout is None else timeout)
        start_rotation = GyroHelper.gyro_rotation()
        target_rotation = start_rotation + (angle if direction == TurnType.RIGHT else -angle)
        while not turn_pid.is_done():
            current_rotation = GyroHelper.gyro_rotation()
            pid_output = turn_pid.compute(target_rotation, current_rotation)

            self.spin(pid_output, -pid_output)

            wait(turn_pid.timestep, SECONDS)

        self.stop(self.stop_mode)
        print("Done Turn: ", turn_pid.get_is_settled(), turn_pid.get_is_timed_out())

        # for log_entry in turn_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)

    def drive_for(self, direction, distance, unit, heading = None, settle_error = None, timeout = None):
        drive_pid = SimplePID(self.drive_pid_constants.Kp, self.drive_pid_constants.Ki, self.drive_pid_constants.Kd)
        drive_pid.set_output_limit(self.drive_pid_constants.max_output) # limit output to 50% power
        drive_pid.set_output_ramp_limit(self.drive_pid_constants.max_ramp)
        # see if we want to override settle and timeout
        if (settle_error is None): drive_pid.set_settle_threshold(self.drive_pid_constants.settle_error)
        else: drive_pid.set_settle_threshold(self.calc_drive_settle_error(settle_error))
        drive_pid.set_timeout(self.default_timeout if timeout is None else timeout)

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
            current_position = (
                (self.left_motors.position(RotationUnits.DEG) - left_start_pos) +
                (self.right_motors.position(RotationUnits.DEG) - right_start_pos)) / 2.0

            pid_output = drive_pid.compute(target_position, current_position)

            turn_pid_output = 0.0
            if (heading is not None):
                current_rotation = GyroHelper.gyro_rotation()
                turn_pid_output = turn_pid.compute(target_rotation, current_rotation)

            self.spin(pid_output + turn_pid_output, pid_output - turn_pid_output)

            wait(drive_pid.timestep, SECONDS)

        self.stop(self.stop_mode)
        print("Done Drive: ", drive_pid.get_is_settled(), drive_pid.get_is_timed_out())

        # for log_entry in drive_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)


    def drive_to_point(self, x, y, tracker):
        pass

    def spin(self, left_speed, right_speed):
        if (self.USE_VOLTAGE):
            left_voltage = self.limit(left_speed * SimpleDrive.MAX_VOLTAGE, SimpleDrive.MAX_VOLTAGE)
            right_voltage = self.limit(right_speed * SimpleDrive.MAX_VOLTAGE, SimpleDrive.MAX_VOLTAGE)

            self.left_motors.spin(FORWARD, left_voltage, VOLT) # type: ignore
            self.right_motors.spin(FORWARD, right_voltage, VOLT) # type: ignore
        else:
            left_percent = self.limit(left_speed * SimpleDrive.MAX_PERCENT, SimpleDrive.MAX_PERCENT)
            right_percent = self.limit(right_speed * SimpleDrive.MAX_PERCENT, SimpleDrive.MAX_PERCENT)

            self.left_motors.spin(FORWARD, left_percent, PERCENT)
            self.right_motors.spin(FORWARD, right_percent, PERCENT)

    def stop(self, mode):
        # Note that setting mode to None will keep motors at their last commanded output
        if (mode is not None):
            self.left_motors.stop(mode)
            self.right_motors.stop(mode)

    def limit(self, input, limit_value):
        if (input > limit_value): return limit_value
        elif (input < -limit_value): return -limit_value
        return input

def demo_print_tracker(tracker: Tracking, x = 0.0, y = 0.0):
    orientation = tracker.get_orientation()
    origin_distance, origin_heading = tracker.trajectory_to_point(x, y)
    print("X: {:.1f} mm, Y: {:.1f} mm, Heading: {:.2f} deg".format(orientation.x, orientation.y, orientation.heading))
    print(" - To Point: Distance: {:.1f} mm, Heading: {:.2f} deg".format(origin_distance, origin_heading))

# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def demo1_drive_straight(drive_train: SimpleDrive, tracker: Tracking):
    demo_print_tracker(tracker)
    drive_train.turn_to_heading(90.0, timeout=2.0)
    demo_print_tracker(tracker)
    drive_train.turn_to_heading(0.0, timeout=2.0)
    demo_print_tracker(tracker)

    distance = 36.0 * 25.4
    timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
    drive_train.drive_for(FORWARD, distance, MM, 0.0,timeout=timeout)
    demo_print_tracker(tracker)
    drive_train.drive_for(REVERSE, distance, MM, 0.0, timeout=timeout)
    demo_print_tracker(tracker)
    
    drive_train.turn_to_heading(0.0, timeout=2.0)
    demo_print_tracker(tracker)

# DEMO2: Once robot has been tuned for a full turn, use this to test turning to specific headings
def demo2_turn_to_headings(drive_train: SimpleDrive, tracker: Tracking):
    headings = [0, 90, 180, 270, 0, 90, 180, 270, 0, 270, 180, 90, 0, 270, 180, 90, 0]
    demo_print_tracker(tracker)
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
        demo_print_tracker(tracker)
        wait(1, SECONDS)

# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def demo3_drive_to_points(drive_train: SimpleDrive, tracker: Tracking):
    demo_print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(90.0, timeout=2.0)
    demo_print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(0.0, timeout=2.0)
    demo_print_tracker(tracker)

    print("Start Drive")
    distance, heading = tracker.trajectory_to_point(36.0 * 25.4, 0.0)
    timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
    demo_print_tracker(tracker, 36.0 * 25.4, 0.0)
    drive_train.turn_to_heading(heading, settle_error=1.0, timeout=0.5)
    drive_train.drive_for(FORWARD, distance, MM, heading, timeout=timeout)
    demo_print_tracker(tracker)

    print("Start Drive")
    distance, heading = tracker.trajectory_to_point(0.0, 0.0)
    heading = Tracking.to_heading(heading + 180.0)
    drive_train.turn_to_heading(heading, settle_error=1.0, timeout=0.5)
    drive_train.drive_for(REVERSE, distance, MM, heading, timeout=timeout)
    demo_print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(0.0)
    demo_print_tracker(tracker)
        
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
    drive_train.set_drive_constants(Kp=0.3, Ki=0.0, Kd=0.0, settle_error=5) # mm
    drive_train.set_heading_lock_constants(Kp=2.0, Ki=0.0, Kd=0.0, settle_error=0.0) # degrees
    drive_train.set_turn_velocity(66, PERCENT)
    drive_train.set_drive_velocity(100, PERCENT)
    drive_train.set_drive_acceleration(10, PERCENT) # 5% per timestep

    tracker = Tracking()
    tracker.set_orientation(Tracking.Orientation(0.0, 0.0, 0.0))

    demo2_turn_to_headings(drive_train, tracker)
    demo1_drive_straight(drive_train, tracker)
    demo3_drive_to_points(drive_train, tracker)

    # place driver control in this while loop
    while True:
        # demo_print_tracker(tracker)
        wait(1, SECONDS)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
