#!/usr/bin/env python3
"""Pure event parsing for the independent GCS mission voice node."""
import re


def mission_event_announcement(node_name, message):
    """Map established guidance logs to stable event keys and phrases."""
    if str(node_name) != "/guidance":
        return None
    text = str(message).strip()
    if text == "Starting waypoint mission":
        return ("mission_start", "Starting the mission. Ready to go.")
    landing = re.match(r"^\[LAND\] Flying to ArUco (\d+) at ", text)
    if landing:
        marker_id = int(landing.group(1))
        return (
            "landing_marker:{}".format(marker_id),
            "Landing sequence started for ArUco marker {}.".format(marker_id),
        )
    if text == "[LAND] Landing complete!":
        return ("mission_complete", "Landing complete. Mission complete.")
    return None


def payload_announcement(actuator):
    """Return the stable event key and phrase for a payload actuator."""
    payloads = {
        "a": ("person", "Payload A deployed for person."),
        "b": ("backpack", "Payload B deployed for backpack."),
    }
    key = str(actuator).lower()
    if key not in payloads:
        return None
    target, phrase = payloads[key]
    return ("payload:{}:{}".format(key, target), phrase)
