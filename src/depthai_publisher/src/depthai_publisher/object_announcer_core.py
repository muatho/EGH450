#!/usr/bin/env python3
"""Pure state and formatting helpers for location-aware target alerts."""
import math
import time


class TargetAnnouncementGate:
    """Confirm targets and suppress repeats for a configurable cooldown."""

    def __init__(self, confirmation_frames=3, match_radius=1.0,
                 deduplicate_by_type=False, cooldown_seconds=60.0):
        if confirmation_frames < 1:
            raise ValueError("confirmation_frames must be at least 1")
        if match_radius <= 0.0:
            raise ValueError("match_radius must be positive")
        if cooldown_seconds <= 0.0:
            raise ValueError("cooldown_seconds must be positive")
        self.confirmation_frames = int(confirmation_frames)
        self.match_radius = float(match_radius)
        self.deduplicate_by_type = bool(deduplicate_by_type)
        self.cooldown_seconds = float(cooldown_seconds)
        self._candidates = []
        self._reported = []

    def reset(self):
        """Forget observations and announcements from the previous mission."""

        self._candidates.clear()
        self._reported.clear()

    @staticmethod
    def _distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _already_reported(self, target_type, position, now):
        return any(
            old_type == target_type
            and (self.deduplicate_by_type
                 or self._distance(old_position, position) <= self.match_radius)
            and now - reported_at < self.cooldown_seconds
            for old_type, old_position, reported_at in self._reported
        )

    def update(self, observations, now=None):
        """Return newly confirmed ``(type, (x, y, z))`` targets."""
        now = time.monotonic() if now is None else float(now)
        self._reported = [
            item for item in self._reported
            if now - item[2] < self.cooldown_seconds
        ]
        observations = [
            (str(kind), tuple(float(value) for value in position))
            for kind, position in observations
        ]
        previous = self._candidates
        used = set()
        candidates = []
        for kind, position in observations:
            matches = [
                (self._distance(item["position"], position), index)
                for index, item in enumerate(previous)
                if index not in used and item["type"] == kind
                and self._distance(item["position"], position) <= self.match_radius
            ]
            if not matches:
                candidates.append({"type": kind, "position": position, "count": 1})
                continue
            _, index = min(matches)
            used.add(index)
            item = previous[index]
            count = item["count"]
            averaged = tuple(
                (old * count + new) / (count + 1)
                for old, new in zip(item["position"], position)
            )
            candidates.append({"type": kind, "position": averaged, "count": count + 1})

        ready = []
        self._candidates = []
        for item in candidates:
            if item["count"] < self.confirmation_frames:
                self._candidates.append(item)
                continue
            target = (item["type"], item["position"])
            if not self._already_reported(target[0], target[1], now):
                self._reported.append((target[0], target[1], now))
                ready.append(target)
        return ready


def announcement_text(target_type, position):
    """Format type and map location in metres to one decimal place."""
    x, y = position[:2]
    return "Detected {} at X {:.1f} metres, Y {:.1f} metres.".format(
        str(target_type).replace("_", " "), x, y
    )


def target_event_announcement(message):
    """Convert an onboard target decision into a short spoken announcement."""

    lines = [line.strip() for line in str(message).strip().splitlines()]

    if len(lines) < 3 or not lines[0].startswith("[TARGET "):
        return None

    event = lines[0][8:-1].strip().upper()
    target = lines[1].split("|", 1)[0].strip().lower()

    payloads = {
        "person": ("EpiPen", "EpiPen"),
        "backpack": ("tracker", "Tracker"),
    }

    if target not in payloads:
        return None

    coordinates = {}

    for field in lines[2].split("|"):
        name, separator, value = field.partition(":")

        if not separator:
            continue

        try:
            coordinates[name.strip()] = float(value.strip())
        except ValueError:
            return None

    if "World X" not in coordinates or "World Y" not in coordinates:
        return None

    display_name = target.title()
    location = "X {:.1f} metres, Y {:.1f} metres".format(
        coordinates["World X"],
        coordinates["World Y"],
    )

    if event == "DETECTED":
        return "{} detected at {}.".format(display_name, location)

    if event == "CONFIRMED":
        return "{} confirmed at {}. Lowering to deploy {}.".format(
            display_name,
            location,
            payloads[target][0],
        )

    if event == "REJECTED":
        return "{} rejected. Returning to search.".format(display_name)

    if event == "IGNORED":
        return "{} ignored. {} already deployed.".format(
            display_name,
            payloads[target][1],
        )

    return None
