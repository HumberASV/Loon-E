"""task.py: central mission state machine for the Loon-E.

Receives:
    Detected objects (label + position) from the ZED object detection topic
    Telemetry (lat, lon, speed, heading) from the Phone Node
    Occupancy grid from SLAM Toolbox (/map)
    Pose in the map frame from SLAM Toolbox (/pose)
    Waypoint list from the Path Planning Node
    Autonomy / e-stop switch levels from the Motor Node

Sends:
    Current and target map cells to the Path Planning Node ('task_path')
    Motor commands [command, target heading, direction] to the Motor Node ('task')

Threading model
---------------
`main()` spins the executor on the main thread; `run_task` runs the mission
state machine on its own thread and only ever *waits on events* -- it never
calls spin_once. The previous version spun the same node from both threads,
which is not safe with rclpy's single-threaded executor (duplicated and dropped
callbacks). Publishing from the worker thread is fine: rclpy publishers are
internally locked.

Shared sensor state is only ever replaced by a single whole-object assignment
(atomic under the GIL), never mutated in place, so the worker thread cannot
observe a half-updated snapshot.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from geometry_msgs.msg import Polygon, Point32, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from zed_msgs.msg import ObjectsStamped


class Task(Node):
    """Runs the selected competition task as a stage-based state machine."""

    # --- Mission tuning -----------------------------------------------------
    # GPS floats never compare equal, so arrival is a radius, not an equality
    # test (this is why the old `arrived()` could never return True).
    ARRIVAL_RADIUS_M = 3.0
    HEADING_TOLERANCE_DEG = 45.0
    # Metres per degree of latitude; longitude is scaled by cos(latitude).
    METERS_PER_DEG_LAT = 111000.0

    # --- Timing -------------------------------------------------------------
    PATH_TIMEOUT_S = 2.0        # give up waiting on path_planning and stop
    RC_STATE_TIMEOUT_S = 1.5    # RC state older than this counts as unsafe
    DOCK_HOLD_S = 10.0          # dwell time in the dock
    DOCK_BACKOUT_S = 10.0       # how long to reverse out of the dock

    # --- Task 1 (buoy gates) ------------------------------------------------
    BUOY_MATCH_RADIUS_M = 2.0   # two sightings this close are the same buoy
    GATE_OFFSET_M = 4.0         # how far to pass to the side of a gate buoy

    # --- Task 2 (collision avoidance) ---------------------------------------
    OTTER_CLEAR_FRACTION = 0.4  # fraction of camera_fov that counts as "clear"
    DETOUR_AHEAD_M = 8.0        # detour waypoint, ahead of the boat
    DETOUR_SIDE_M = 5.0         # detour waypoint, offset to starboard

    def __init__(self):
        """Initialize the Task node, its topics, parameters and worker thread."""
        # Node name matches the `/task:` block in config.yaml so the parameters
        # apply when the node is run without the launch file's `name=` override.
        super().__init__('task')

        # Events
        self.object_data_ready_event = threading.Event()
        self.phone_data_ready_event = threading.Event()
        self.position_data_ready_event = threading.Event()
        self.map_data_ready_event = threading.Event()
        self.path_data_ready_event = threading.Event()
        # Set once on shutdown so the worker thread can exit promptly.
        self.stop_event = threading.Event()

        # Declare parameters with fallback/default values
        # `task` is a scalar, not a list: the old [1] default meant the dispatch
        # `match` compared a list against ints and never matched any case, so
        # run_task looped forever doing nothing. 31/32 encode tasks 3a/3b,
        # because a float in an int list changes the whole parameter's type.
        self.declare_parameter('task', 1)
        self.declare_parameter('latitude', [63.4420522, 63.442353, 63.4424279])
        self.declare_parameter('longitude', [10.4239289, 10.4237452, 10.4243540])
        self.declare_parameter('camera_fov', 40.0)
        # The ZED topic embeds the camera_name/zed_node_name launch arguments,
        # so it has to be configurable rather than hard-coded.
        self.declare_parameter('objects_topic', '/zedx/zed_node/obj_det/objects')
        # SAFETY: the sense of the two switch inputs is set here rather than
        # baked into the comparison, so it can be corrected from YAML after a
        # bench test instead of needing a rebuild. Defaults follow motor.py's
        # LED logic (auto engaged = line high, e-stop clear = line high).
        self.declare_parameter('auto_active_high', True)
        self.declare_parameter('estop_clear_high', True)

        # Retrieve Parameters
        self.task = self.get_parameter('task').value
        latitude = self.get_parameter('latitude').value
        longitude = self.get_parameter('longitude').value
        self.get_path(latitude, longitude)  # Merge lists
        self.camera_fov = float(self.get_parameter('camera_fov').value)
        objects_topic = self.get_parameter('objects_topic').value
        self.auto_active_high = self.get_parameter('auto_active_high').value
        self.estop_clear_high = self.get_parameter('estop_clear_high').value

        # Publishers and Subscribers
        self.path_pub = self.create_publisher(Polygon, 'task_path', 10)
        # Was 'task_motor', which motor.py does not subscribe to -- every motor
        # command, including the shutdown stop, went nowhere.
        self.motor_pub = self.create_publisher(Float32MultiArray, 'task', 10)
        # ZED provides both labels and positions, so there is no separate
        # locations topic.
        self.objects_sub = self.create_subscription(
            ObjectsStamped, objects_topic, self.object_callback, 10)
        self.phone_sub = self.create_subscription(
            Float32MultiArray, 'phone', self.phone_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        # SLAM Toolbox publishes /pose as PoseWithCovarianceStamped. Subscribing
        # as PoseStamped meant the types never matched, no callback ever fired,
        # and run_task blocked forever waiting for position data.
        self.position_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.position_callback, 10)
        self.path_sub = self.create_subscription(
            Polygon, 'path', self.path_callback, 10)
        # motor.py owns board.D12/D13 and republishes their levels here. Two
        # processes opening the same gpiochip line fails with "Device or
        # resource busy" under libgpiod, so this node no longer touches the
        # hardware itself.
        self.rc_sub = self.create_subscription(
            Int8MultiArray, 'rc_state', self.rc_callback, 10)

        # Other internal variables
        self.i = 0                  # Index of the current target in self.path
        self.stage = 0              # Current stage within the task
        self.finished = False       # Set once the course is complete
        self.gate_was_open = False  # Tracks autonomy-gate edges so we can stop
        self.stage_deadline = None  # Non-blocking replacement for time.sleep
        self.buoy_list = []         # Buoys already used to adjust the course
        self.target_heading = None  # Latched heading target for the dock turns

        # Other variables from topics. None means "not received yet" so the
        # sentinel -999 values can no longer leak into the navigation maths.
        self.position = None        # Current GPS position as [lat, lon]
        self.heading = None         # Current heading, degrees, 0 = North
        self.speed = None           # Current speed
        self.detections = []        # [(label, forward_m, lateral_m), ...]
        self.res = None             # Map resolution, metres per cell
        self.origin = None          # Map origin as [y, x] in metres
        self.map = None             # Occupancy grid
        self.global_position = None # Current position as [row, col] map cells
        self.waypoints = []         # Path to destination, as [row, col] cells
        self.auto_engaged = False   # Autonomy switch, after polarity mapping
        self.estop_clear = False    # E-stop switch, after polarity mapping
        self.rc_state_time = 0.0    # monotonic timestamp of the last rc_state

        # Thread
        self.receiver = threading.Thread(target=self.run_task, daemon=True)
        self.receiver.start()

    def get_path(self, latitude, longitude):
        """Setup function which converts a list of latitudes and a list of
        longitudes into a list of [latitude, longitude].

        Args:
            latitude: List of latitudes
            longitude: List of longitudes

        Raises:
            ValueError: If the lists differ in length, or are empty.
        """
        # A bare `raise` here previously produced "No active exception to
        # reraise" instead of the documented ValueError.
        if len(latitude) != len(longitude):
            self.get_logger().error("Latitude and longitude lists are not of same size")
            raise ValueError("latitude and longitude must be the same length")

        if not latitude:
            self.get_logger().error("No GPS waypoints configured")
            raise ValueError("latitude/longitude parameters are empty")

        self.path = [[latitude[i], longitude[i]] for i in range(len(latitude))]

    # ROS - Publish
    def publish_path(self, data):
        """Publish a path request as a Polygon message.

        Args:
            data: Message content in the form [start cell, target cell],
                each cell being [row, col].
        """
        msg = Polygon()

        for position in data:
            point = Point32()
            # Point32 fields are float32 and rclpy asserts isinstance(v, float).
            # numpy integers are not a float subclass, so the raw cell values
            # used to raise on assignment -- hence the explicit float().
            point.y = float(position[0])
            point.x = float(position[1])
            msg.points.append(point)

        self.path_pub.publish(msg)
        # Polygon carries points, not data; the old msg.data raised AttributeError.
        self.get_logger().info(f"Path request: {data}")

    def publish_motor(self, data):
        """Publish a motor command as a Float32MultiArray message.

        Args:
            data: Message content in the form [command, target heading, direction].
                Where not applicable the value is -999.
        """
        msg = Float32MultiArray()
        msg.data = [float(value) for value in data]
        self.motor_pub.publish(msg)
        self.get_logger().info(f"Motor: {msg.data}")

    # General Code - Geometry helpers
    @staticmethod
    def wrap180(angle):
        """Wrap an angle in degrees to [-180, 180)."""
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def wrap360(angle):
        """Wrap an angle in degrees to [0, 360)."""
        return angle % 360.0

    def offset_meters(self, start, end):
        """Return the (north, east) offset in metres from start to end.

        Args:
            start: [latitude, longitude] of the first point.
            end: [latitude, longitude] of the second point.
        """
        # The cosine scaling needs the mean *latitude*; the old code averaged
        # index 1, which is longitude, and shrank every east/west distance by an
        # arbitrary factor.
        lat_ref = math.radians((start[0] + end[0]) / 2.0)
        north = (end[0] - start[0]) * self.METERS_PER_DEG_LAT
        east = (end[1] - start[1]) * self.METERS_PER_DEG_LAT * math.cos(lat_ref)
        return north, east

    def body_to_gps(self, forward, lateral):
        """Convert an offset measured from the boat into a GPS coordinate.

        The camera reports objects relative to the hull, so the offset has to be
        rotated by the current heading before it means anything in latitude and
        longitude. The old code fed camera-frame metres straight into
        get_coordinate as if they were map cells, which both skipped this
        rotation and multiplied the offset by the map resolution.

        Args:
            forward: Metres ahead of the boat (ZED x).
            lateral: Metres to port of the boat (ZED y).

        Returns:
            [latitude, longitude] of the offset point.
        """
        hdg = math.radians(self.heading)
        # Port is heading - 90 degrees: cos(h-90) = sin(h), sin(h-90) = -cos(h).
        north = forward * math.cos(hdg) + lateral * math.sin(hdg)
        east = forward * math.sin(hdg) - lateral * math.cos(hdg)

        lat = self.position[0] + north / self.METERS_PER_DEG_LAT
        lon = self.position[1] + east / (
            self.METERS_PER_DEG_LAT * math.cos(math.radians(self.position[0])))
        return [lat, lon]

    def get_map_cell(self, start, end):
        """Get the map cell of a GPS coordinate.

        Args:
            start: Current GPS coordinate.
            end: Target GPS coordinate.

        Returns:
            Target coordinate in the global map, as [row, col].

        Possible edit: Use haversine formula instead
        """
        north, east = self.offset_meters(start, end)
        # The offset is start -> end and is *added* to the current cell. The old
        # code computed end -> start and added that, mirroring every goal
        # through the boat's position and planning in the opposite direction.
        row = self.global_position[0] + round(north / self.res)
        col = self.global_position[1] + round(east / self.res)
        return [row, col]

    def get_coordinate(self, current_position, waypoints):
        """Get the GPS position and compass heading of the next waypoint.

        Args:
            current_position: Current GPS coordinate.
            waypoints: Global map cells representing the path to the destination.

        Returns:
            target_position: GPS coordinate of the target position
            heading: Heading of the coordinate relative to North, in [0, 360)

        Possible edit: Use haversine formula instead
        """
        # Waypoints are stored [row, col] i.e. [north, east] (see path_callback).
        north = (waypoints[1][0] - waypoints[0][0]) * self.res
        east = (waypoints[1][1] - waypoints[0][1]) * self.res

        # A compass bearing is measured from North towards East, so it is
        # atan2(east, north). The old atan2(north, east) produced an angle
        # measured from East, which made cos()/sin() below fill in the latitude
        # from the east offset and vice versa.
        heading = math.atan2(east, north)

        d = math.hypot(north, east)
        d_lat = d * math.cos(heading) / self.METERS_PER_DEG_LAT
        d_long = d * math.sin(heading) / (
            self.METERS_PER_DEG_LAT * math.cos(math.radians(current_position[0])))

        target_position = [current_position[0] + d_lat, current_position[1] + d_long]
        return target_position, self.wrap360(math.degrees(heading))

    # General Code - Subprograms
    def find_detection(self, label):
        """Return the closest detection with the given label, or None.

        Returns:
            (label, forward_m, lateral_m) or None if the label is not in frame.
        """
        # Iterating the *labels* and using one as a list index was the old bug
        # here; matching detections are selected by nearest forward distance.
        matches = [d for d in self.detections if d[0] == label]
        return min(matches, key=lambda d: d[1]) if matches else None

    def object_found(self, target_objects):
        """Checks if all of the given objects are in the current image frame.

        Args:
            target_objects: Iterable of labels to identify. Passing a bare
                string iterates its characters, so callers must pass a list.

        Returns True if every object was found and False otherwise.
        """
        if not target_objects:
            return False

        labels = [d[0] for d in self.detections]
        return all(target in labels for target in target_objects)

    def arrived(self, i):
        """Checks if the Loon-E is at the desired GPS point.

        Args:
            i: Desired index for the GPS point in the list provided by self.path.

        Returns True if the Loon-E is within ARRIVAL_RADIUS_M and False otherwise.
        """
        if self.position is None or not 0 <= i < len(self.path):
            return False

        north, east = self.offset_meters(self.position, self.path[i])
        return math.hypot(north, east) <= self.ARRIVAL_RADIUS_M

    def check_heading(self):
        """Checks if the Loon-E is within 45 degrees of self.target_heading.
        Turns toward the target heading if this is not the case.

        Returns True if the Loon-E is within 45 degrees and False otherwise.
        """
        if self.heading is None or self.target_heading is None:
            return False

        # wrap180 replaces the old self.convert() calls, which were only ever
        # defined on MotorNode and raised AttributeError here.
        d_heading = self.wrap180(self.heading - self.target_heading)

        if abs(d_heading) > self.HEADING_TOLERANCE_DEG:
            # dir = +1 means turn left in motor.turn_in_place, which is the
            # correct way to reduce a heading that is above the target.
            self.publish_motor([2.0, -999.0, math.copysign(1.0, d_heading)])
            return False

        # Nothing else cancels the turn command, so stop here or the boat keeps
        # spinning after it is lined up.
        self.stop()
        return True

    def buoy_in_list(self, coord):
        """Checks whether a buoy at this GPS coordinate has already been used."""
        for buoy in self.buoy_list:
            # The old comparison chained a *list* against floats
            # (buoy[0] < coord < buoy[0]) and raised TypeError on first use.
            north, east = self.offset_meters(buoy, coord)
            if math.hypot(north, east) < self.BUOY_MATCH_RADIUS_M:
                return True

        return False

    def sensors_ready(self):
        """True when everything drive() needs has been received at least once."""
        return (self.position is not None
                and self.heading is not None
                and self.global_position is not None
                and self.res)

    def advance_waypoint(self):
        """Step to the next GPS waypoint, stopping once the course is done.

        Returns True while there is still somewhere to go.
        """
        self.i += 1
        if self.i >= len(self.path):
            self.get_logger().info("Final waypoint reached - course complete.")
            self.shutdown()
            return False

        return True

    def start_stage_timer(self, seconds):
        """Arm a non-blocking stage timer.

        The dock stages used time.sleep(10), which froze the whole state machine
        -- including the e-stop check -- for ten seconds of live motion.
        """
        self.stage_deadline = time.monotonic() + seconds

    def stage_timer_done(self):
        """True once the timer armed by start_stage_timer has expired."""
        return self.stage_deadline is not None and time.monotonic() >= self.stage_deadline

    def drive(self):
        """Self-driving control loop."""
        if not self.sensors_ready():
            return

        # Calculate the target position in the global map
        target_position = self.get_map_cell(self.position, self.path[self.i])

        # Cleared *before* publishing: clearing afterwards let a path left over
        # from the previous cycle satisfy the wait below.
        self.path_data_ready_event.clear()
        self.publish_path([self.global_position, target_position])

        # Wait for path planning to complete. The old unbounded wait meant that
        # if path_planning was down the boat coasted on its last command
        # forever; on timeout we stop instead.
        if not self.path_data_ready_event.wait(timeout=self.PATH_TIMEOUT_S):
            self.get_logger().warning("No path received from path_planning; stopping.")
            self.stop()
            return

        # Send the command to the Motor Node. get_coordinate indexes
        # waypoints[1], so a shorter path is not usable.
        if len(self.waypoints) >= 2:
            target_heading = self.get_coordinate(self.position, self.waypoints)[1]
            self.publish_motor([1.0, target_heading, -999.0])
        else:
            self.get_logger().warning("Path too short to steer on; stopping.")
            self.stop()

    def stop(self):
        """Command the motors to neutral."""
        self.publish_motor([0.0, -999.0, -999.0])

    def shutdown(self):
        """Stops the motors at the end of the task or on KeyboardInterrupt."""
        self.finished = True
        self.stop_event.set()
        self.stop()

    # General Code - Tasks
    def task_0(self):
        """For PID testing"""
        self.publish_motor([1.0, 170.0, -999.0])

    def task_1(self):
        """Maneuvering and Path Finding.

        Follows the GPS course, nudging the current waypoint sideways so the
        boat passes on the correct side of any red or green gate buoy.
        """
        match self.stage:
            case 0:  # Drive
                self.drive()

                # Pick the nearest gate buoy of either colour. The old search
                # seeded the "best so far" distance at 0 and kept candidates
                # closer than that, so it could never select anything.
                red = self.find_detection('red')
                green = self.find_detection('green')
                candidates = [b for b in (red, green) if b is not None]
                buoy = min(candidates, key=lambda b: b[1]) if candidates else None

                if buoy is not None and self.sensors_ready():
                    label, forward, lateral = buoy
                    # Offset perpendicular to the buoy rather than in fixed
                    # compass steps: red is left to starboard (so aim to port of
                    # it), green is left to port.
                    side = self.GATE_OFFSET_M if label == 'red' else -self.GATE_OFFSET_M
                    coord = self.body_to_gps(forward, lateral + side)

                    if not self.buoy_in_list(coord):
                        self.path[self.i] = coord
                        self.buoy_list.append(coord)

                if self.arrived(self.i):
                    self.advance_waypoint()

    def task_2(self):
        """Collision Avoidance.

        Holds the GPS course but reacts to the Otter: ignore it when it is well
        off to one side, give way when it is to starboard, and alter course to
        starboard when it is head on.
        """
        # Half the visible width counts as "clear to one side".
        clear_m = self.camera_fov * self.OTTER_CLEAR_FRACTION / 2.0

        match self.stage:
            case 0:  # Go to destination, checking for the Otter
                # find_detection replaces check_change('otter', True), which
                # iterated the characters of the string and never matched.
                otter = self.find_detection('otter')

                if otter is None or not self.sensors_ready():
                    self.drive()
                elif abs(otter[2]) >= clear_m:
                    # Well off to one side; nothing to do. The old pair of
                    # thresholds (y > 16 or y < 24) overlapped, so every value
                    # took one of these branches and the head-on case below was
                    # unreachable.
                    self.stage = 20
                elif otter[2] < 0:
                    # To starboard: we are the give-way vessel, so hold station.
                    self.stage = 12
                else:
                    # Head on or to port: alter course to starboard. insert()
                    # takes (index, value) -- the old insert(coord, 1) inserted
                    # the integer 1 at position `coord`.
                    detour = self.body_to_gps(self.DETOUR_AHEAD_M, -self.DETOUR_SIDE_M)
                    self.path.insert(self.i, detour)
                    self.stage = 11

            case 11:  # Run the detour, then resume the course
                self.drive()

                if self.arrived(self.i):
                    if self.advance_waypoint():
                        self.stage = 20

            case 12:  # Otter to starboard, wait until it passes
                self.stop()

                if self.find_detection('otter') is None:
                    self.stage = 20

            case 20:  # Go to destination
                self.drive()

                if self.arrived(self.i):
                    self.advance_waypoint()

    def task_3a(self):
        """Forward Docking.

        TODO(team): the AR-tag scan that should generate the bay waypoints is
        not implemented. Until it is, the bay approach and exit have to be
        supplied as ordinary latitude/longitude parameters; the stages below
        step through them instead of crashing on a missing waypoint as the old
        unconditional `self.i += 1` did.
        """
        match self.stage:
            case 0:  # Run in to the bay entrance
                self.drive()

                if self.arrived(self.i):
                    if self.advance_waypoint():
                        self.stage = 10

            case 10:  # Nose into the bay
                self.drive()

                if self.arrived(self.i):
                    self.stop()
                    self.start_stage_timer(self.DOCK_HOLD_S)
                    self.stage = 11

            case 11:  # Sit in the bay for the required dwell time
                if self.stage_timer_done():
                    self.publish_motor([-1.0, -999.0, -999.0])  # reverse out
                    self.start_stage_timer(self.DOCK_BACKOUT_S)
                    self.stage = 12

            case 12:  # Reverse clear of the bay
                if self.stage_timer_done():
                    self.stop()
                    if self.advance_waypoint():
                        self.stage = 20

            case 20:  # Head for the exit waypoint
                self.drive()

                if self.arrived(self.i):
                    self.advance_waypoint()

    def task_3b(self):
        """Parallel Docking.

        TODO(team): as with task_3a, the AR-tag scan is not implemented and the
        slip waypoint comes from the GPS parameters.
        """
        match self.stage:
            case 0:  # Run in to the slip entrance
                self.drive()

                if self.arrived(self.i):
                    if self.advance_waypoint():
                        self.stage = 10

            case 10:  # Run up alongside the slip
                self.drive()

                if self.arrived(self.i):
                    # self.heading, not self.current_heading, which never
                    # existed on this node. Latched once here: recomputing
                    # target = current + 90 on every pass (as the old case 20
                    # did) made the turn stages chase each other forever.
                    self.target_heading = self.wrap360(self.heading + 90.0)
                    self.stage = 20

            case 20:  # Swing 90 degrees to lie parallel to the dock
                if self.check_heading():
                    self.start_stage_timer(self.DOCK_HOLD_S)
                    self.stage = 30

            case 30:  # Hold alongside for the required dwell time
                if self.stage_timer_done():
                    # Second 90 degree turn, off the latched target rather than
                    # the live heading, to point back out of the slip.
                    self.target_heading = self.wrap360(self.target_heading + 90.0)
                    self.stage = 40

            case 40:  # Complete the turn out
                if self.check_heading():
                    self.stage = 50

            case 50:  # Leave the dock
                self.drive()

                if self.arrived(self.i):
                    self.advance_waypoint()

    def task_4(self):
        """Surprise Task"""
        return

    # General Code - Main loop
    def gate_open(self):
        """True when the boat is in autonomous mode with the e-stop clear.

        Fails closed if motor.py has not reported recently, so losing the node
        that owns the switches stops the boat rather than leaving it running
        blind.
        """
        if time.monotonic() - self.rc_state_time > self.RC_STATE_TIMEOUT_S:
            return False

        return self.auto_engaged and self.estop_clear

    def wait_for_data(self):
        """Block until fresh sensor data is available and the boat is armed.

        Returns False if the node is shutting down.
        """
        while not self.stop_event.is_set():
            if not self.gate_open():
                # SAFETY: the old loop simply stopped iterating when the gate
                # closed, which left the last drive command latched in the
                # motor node. Command a stop on the closing edge instead.
                if self.gate_was_open:
                    self.get_logger().warning("Autonomy gate closed - stopping motors.")
                    self.stop()
                    self.gate_was_open = False

                self.stop_event.wait(0.05)
                continue

            if (self.object_data_ready_event.is_set()
                    and self.phone_data_ready_event.is_set()
                    and self.position_data_ready_event.is_set()):
                self.gate_was_open = True
                return True

            # Waiting on the stop event rather than sleeping keeps shutdown
            # responsive. This thread must not spin the executor: main() does.
            self.stop_event.wait(0.02)

        return False

    def run_task(self):
        """Wait for data, execute one pass of the task, then clear the events."""
        while not self.stop_event.is_set():
            if not self.wait_for_data():
                return

            if self.finished:
                return

            self.get_logger().debug("Ready!")

            # self.task is a scalar int; 31/32 stand in for tasks 3a/3b.
            match self.task:
                case 0: self.task_0()
                case 1: self.task_1()
                case 2: self.task_2()
                case 31: self.task_3a()
                case 32: self.task_3b()
                case 4: self.task_4()
                case _:
                    self.get_logger().error(f"Unknown task {self.task}; stopping.")
                    self.shutdown()
                    return

            # Clear to wait until the next set of data is received
            self.object_data_ready_event.clear()
            self.phone_data_ready_event.clear()
            self.position_data_ready_event.clear()

    # ROS - Subscribe
    def object_callback(self, msg: ObjectsStamped) -> None:
        """Callback for the ZED object detection subscription.

        Args:
            msg (ObjectsStamped): The ZED message containing detected objects.
        """
        # Label and position are stored together in ONE list, replaced in a
        # single assignment. The old pair of parallel lists could be read
        # between the two writes, pairing a label from one frame with a position
        # from the next.
        # ZED positions follow REP-103: x forward, y to port, in metres.
        self.detections = [
            (obj.label, float(obj.position[0]), float(obj.position[1]))
            for obj in msg.objects
        ]
        self.get_logger().debug(f"Detections: {self.detections}")
        self.object_data_ready_event.set()

    def phone_callback(self, msg: Float32MultiArray) -> None:
        """Handle incoming phone telemetry and update position, speed and heading.

        Args:
            msg: Float32MultiArray where
                index 0 is latitude
                index 1 is longitude
                index 2 is speed
                index 3 is heading.
        """
        data = msg.data
        if len(data) < 4:
            self.get_logger().warning(f"Short phone message: {list(data)}")
            return

        # Assigned as a whole list so the worker thread never sees a half
        # updated fix.
        self.position = [data[0], data[1]]
        self.speed = data[2]
        self.heading = self.wrap360(data[3])
        self.get_logger().debug(f"Phone: {list(data)}")
        self.phone_data_ready_event.set()

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Handle incoming occupancy grids and update the map metadata."""
        self.res = msg.info.resolution
        # Plain floats: the old int numpy array truncated the origin to whole
        # metres and shifted every cell computation.
        self.origin = [msg.info.origin.position.y, msg.info.origin.position.x]
        # The grid itself is now kept -- task_1 and task_2 referenced self.map,
        # which nothing ever assigned.
        self.map = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.get_logger().debug(
            f"Map {msg.info.height}x{msg.info.width} @ {self.res} m/cell")
        self.map_data_ready_event.set()

    def position_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Handle incoming poses and update the current position in the global map.

        Args:
            msg: PoseWithCovarianceStamped from SLAM Toolbox, in the map frame.
        """
        # `if self.map_data_ready_event:` was always True (an Event object is
        # truthy), so this ran with res = -999 and produced garbage cells.
        if not self.map_data_ready_event.is_set():
            return

        pose = msg.pose.pose
        self.global_position = [
            int(round((pose.position.y - self.origin[0]) / self.res)),
            int(round((pose.position.x - self.origin[1]) / self.res)),
        ]
        self.get_logger().debug(f"Position: {self.global_position}")
        self.position_data_ready_event.set()

    def path_callback(self, msg: Polygon) -> None:
        """Handle incoming waypoint lists.

        Args:
            msg: Polygon of waypoints.
        """
        # Polygon paths carry waypoints in points, not data.
        self.waypoints = [[p.y, p.x] for p in msg.points]
        self.get_logger().debug(f"Waypoints: {self.waypoints}")
        self.path_data_ready_event.set()

    def rc_callback(self, msg: Int8MultiArray) -> None:
        """Handle the autonomy and e-stop switch levels published by motor.py.

        Args:
            msg: Int8MultiArray of raw line levels where
                index 0 is the autonomy switch
                index 1 is the e-stop switch.
        """
        if len(msg.data) < 2:
            return

        # Raw levels are mapped through the polarity parameters so the sense can
        # be corrected in YAML after a bench test.
        auto_raw = bool(msg.data[0])
        estop_raw = bool(msg.data[1])
        self.auto_engaged = auto_raw if self.auto_active_high else not auto_raw
        self.estop_clear = estop_raw if self.estop_clear_high else not estop_raw
        self.rc_state_time = time.monotonic()


def main(args=None):
    """Main function to initialize the ROS2 node and start spinning."""
    rclpy.init(args=args)
    task = Task()
    try:
        # The executor runs here and only here; run_task waits on events rather
        # than spinning the same node from its own thread.
        rclpy.spin(task)
    except KeyboardInterrupt:
        task.get_logger().info("Task node interrupted by user.")
    finally:
        # Publish the stop while the context is still up, then tear down.
        task.shutdown()
        task.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
