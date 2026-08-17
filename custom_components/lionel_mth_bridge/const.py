"""Constants for the O Gauge Command Control Bridge integration."""

import json
import os

DOMAIN = "lionel_mth_bridge"

CONF_HOST = "host"
CONF_PORT = "port"

DEFAULT_PORT = 8580
DEFAULT_SCAN_INTERVAL = 30

# Endpoint paths
STATUS_ENDPOINT = "/status"
REFRESH_ENDPOINT = "/refresh"
RENAME_ENDPOINT = "/rename"


def _load_version() -> str:
    """Read the integration version from manifest.json."""
    try:
        manifest_path = os.path.join(os.path.dirname(__file__), "manifest.json")
        with open(manifest_path) as f:
            return json.load(f).get("version", "unknown")
    except Exception:
        return "unknown"


SW_VERSION = _load_version()
