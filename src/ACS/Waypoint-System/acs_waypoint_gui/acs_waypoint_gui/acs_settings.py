"""Persistent settings for ACS GUI."""

import json
import os

_SETTINGS_PATH = os.path.join(os.path.expanduser('~'), '.acs_waypoint_gui_settings.json')
_DEFAULTS = {
    'job_file': '',
    'max_speed': '0.3',
    'accel': '0.3',
    'start_wp': '1',
    'end_wp': '0',
    'map_file': '',
}


def load_settings() -> dict:
    if not os.path.exists(_SETTINGS_PATH):
        return None
    try:
        with open(_SETTINGS_PATH, 'r') as f:
            data = json.load(f)
        for k, v in _DEFAULTS.items():
            data.setdefault(k, v)
        return data
    except Exception:
        return None


def save_settings(settings: dict):
    try:
        with open(_SETTINGS_PATH, 'w') as f:
            json.dump(settings, f, indent=2)
    except Exception:
        pass
