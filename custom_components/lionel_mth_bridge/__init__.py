# pylint: disable=import-error
"""The O Gauge Command Control Bridge integration.

Polls the bridge's HTTP status endpoint every 30 seconds and exposes
connection status and engine library data as Home Assistant entities.
"""

from __future__ import annotations

import logging
from datetime import timedelta
from typing import Any

import aiohttp
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant
from homeassistant.helpers.aiohttp_client import async_get_clientsession
from homeassistant.helpers.update_coordinator import (
    DataUpdateCoordinator,
    UpdateFailed,
)

from .const import (
    DEFAULT_SCAN_INTERVAL,
    DOMAIN,
    REFRESH_ENDPOINT,
    RENAME_ENDPOINT,
    STATUS_ENDPOINT,
)

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up O Gauge Command Control Bridge from a config entry."""
    host = entry.data[CONF_HOST]
    port = entry.data.get(CONF_PORT, 8580)

    coordinator = LionelMthBridgeCoordinator(hass, host, port)
    await coordinator.async_config_entry_first_refresh()

    hass.data.setdefault(DOMAIN, {})[entry.entry_id] = coordinator

    await hass.config_entries.async_forward_entry_setups(entry, ["sensor", "button", "text"])

    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    unload_ok = await hass.config_entries.async_unload_platforms(entry, ["sensor", "button", "text"])
    if unload_ok:
        hass.data[DOMAIN].pop(entry.entry_id)
    return unload_ok


class LionelMthBridgeCoordinator(DataUpdateCoordinator):
    """Coordinator that polls the bridge HTTP endpoint."""

    def __init__(self, hass: HomeAssistant, host: str, port: int) -> None:
        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            update_interval=timedelta(seconds=DEFAULT_SCAN_INTERVAL),
        )
        self.host = host
        self.port = port
        self.base_url = f"http://{host}:{port}"

    async def _async_update_data(self) -> dict[str, Any]:
        """Fetch status data from the bridge."""
        url = f"{self.base_url}{STATUS_ENDPOINT}"
        try:
            session = async_get_clientsession(self.hass)
            async with session.get(
                url, timeout=aiohttp.ClientTimeout(total=10)
            ) as resp:
                if resp.status != 200:
                    raise UpdateFailed(f"HTTP {resp.status} from bridge")
                return await resp.json()
        except aiohttp.ClientError as err:
            raise UpdateFailed(f"Connection error: {err}")
        except Exception as err:
            raise UpdateFailed(f"Error fetching bridge status: {err}")

    async def async_refresh_engines(self) -> bool:
        """Send POST /refresh to trigger an engine library rescan."""
        url = f"{self.base_url}{REFRESH_ENDPOINT}"
        try:
            session = async_get_clientsession(self.hass)
            async with session.post(
                url, timeout=aiohttp.ClientTimeout(total=10)
            ) as resp:
                return resp.status == 200
        except Exception:
            return False

    async def async_set_engine_name(
        self, engine_type: str, engine_id: int, name: str
    ) -> bool:
        """Send POST /rename to override an engine name on the bridge.

        Pass an empty string to clear the override and revert to the polled name.
        """
        url = f"{self.base_url}{RENAME_ENDPOINT}"
        payload = {"type": engine_type, "id": engine_id, "name": name}
        try:
            session = async_get_clientsession(self.hass)
            async with session.post(
                url, json=payload, timeout=aiohttp.ClientTimeout(total=10)
            ) as resp:
                if resp.status == 200:
                    # Force a refresh so the new name shows up immediately
                    await self.async_request_refresh()
                    return True
            return False
        except Exception:
            return False
