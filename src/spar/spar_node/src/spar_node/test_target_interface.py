"""Explicit test-only target injection using the shared obstacle map."""

import math
import random
import threading

import rospy

from nav_msgs.msg import OccupancyGrid
from spar_node.srv import InjectTestTarget, InjectTestTargetResponse


class TestTargetInterface:
    """Validate injection commands and select free map locations."""

    PERSON_CLASS_ID = 0
    BACKPACK_CLASS_ID = 1
    CLASS_NAMES = {
        "person": PERSON_CLASS_ID,
        "backpack": BACKPACK_CLASS_ID,
    }

    def __init__(self):
        self._enabled = rospy.get_param("~enable_test_target_injection", False)
        self._grid_topic = rospy.get_param("~test_target_grid_topic", "/grid")
        self._obstacle_clearance = rospy.get_param("~test_target_obstacle_clearance", 0.3)
        self._maximum_distance = rospy.get_param("~test_target_maximum_distance", 2.0)
        self._lock = threading.RLock()
        self._grid = None
        self._request_handler = None

        if not math.isfinite(self._obstacle_clearance) or self._obstacle_clearance < 0:
            raise ValueError("~test_target_obstacle_clearance must be finite and non-negative")
        if not math.isfinite(self._maximum_distance) or self._maximum_distance <= 0:
            raise ValueError("~test_target_maximum_distance must be finite and greater than zero")

        self._grid_subscriber = rospy.Subscriber(self._grid_topic, OccupancyGrid, self._grid_callback, queue_size=1)
        self._service = None

        if self._enabled:
            self._service = rospy.Service("/mission/inject_test_target", InjectTestTarget, self._handle_request)
            rospy.logwarn("Test target injection is ENABLED; accepted commands can move the aircraft and deploy a payload")
        else:
            rospy.loginfo("Test target injection is disabled")

    @property
    def enabled(self) -> bool:
        """Return whether the deliberately gated test service is available."""
        return self._enabled

    def set_request_handler(self, handler):
        """Register the mission-policy callback used for accepted requests."""
        self._request_handler = handler

    def _grid_callback(self, message: OccupancyGrid):
        """Retain the latest latched occupancy grid used by Breadcrumb."""
        with self._lock:
            self._grid = message

    def random_free_location(self, x_min: float, x_max: float, y_min: float, y_max: float, current_x: float, current_y: float):
        """Choose a nearby free grid-cell centre inside the search footprint."""
        with self._lock:
            grid = self._grid

        if grid is None:
            return None

        resolution = grid.info.resolution
        width = grid.info.width
        height = grid.info.height
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        if resolution <= 0 or width == 0 or height == 0 or len(grid.data) != width * height:
            rospy.logerr("Cannot inject a test target because /grid is malformed")
            return None

        clearance_cells = int(math.ceil(self._obstacle_clearance / resolution))
        free_locations = []

        for cell_y in range(height):
            y = origin_y + (cell_y + 0.5) * resolution
            if y < y_min or y > y_max:
                continue

            for cell_x in range(width):
                x = origin_x + (cell_x + 0.5) * resolution
                if x < x_min or x > x_max:
                    continue

                if math.hypot(x - current_x, y - current_y) > self._maximum_distance:
                    continue

                if self._cell_has_clearance(grid, cell_x, cell_y, clearance_cells):
                    free_locations.append((x, y))

        if not free_locations:
            return None

        return random.SystemRandom().choice(free_locations)

    @staticmethod
    def _cell_has_clearance(grid: OccupancyGrid, cell_x: int, cell_y: int, clearance_cells: int) -> bool:
        """Return whether a cell and its requested neighbourhood are unoccupied."""
        width = grid.info.width
        height = grid.info.height

        for nearby_y in range(cell_y - clearance_cells, cell_y + clearance_cells + 1):
            for nearby_x in range(cell_x - clearance_cells, cell_x + clearance_cells + 1):
                if nearby_x < 0 or nearby_x >= width or nearby_y < 0 or nearby_y >= height:
                    return False

                # Treat unknown cells (-1) as unsafe as well as occupied cells.
                if grid.data[nearby_x + nearby_y * width] != 0:
                    return False

        return True

    def _handle_request(self, request):
        """Convert a person/backpack command into a guarded mission request."""
        target_class = request.target_class.strip().lower()
        class_id = self.CLASS_NAMES.get(target_class)

        if class_id is None:
            return InjectTestTargetResponse(False, "target_class must be 'person' or 'backpack'", 0.0, 0.0)

        if self._request_handler is None:
            return InjectTestTargetResponse(False, "mission controller is not ready", 0.0, 0.0)

        accepted, message, location = self._request_handler(class_id, target_class)
        x, y = location if location is not None else (0.0, 0.0)
        return InjectTestTargetResponse(accepted, message, x, y)
