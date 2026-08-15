"""Button platform for the Lionel MTH Bridge integration.

Provides a "Refresh Engines" button that triggers a rescan of both
the WTIU (track poll) and Base 3 (library query) engine libraries.
"""

from __future__ import annotations

import logging

from homeassistant.components.button import ButtonEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import LionelMthBridgeCoordinator
from .const import DOMAIN, SW_VERSION

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up Lionel MTH Bridge buttons from a config entry."""
    coordinator: LionelMthBridgeCoordinator = hass.data[DOMAIN][entry.entry_id]
    host = entry.data[CONF_HOST]
    port = entry.data.get(CONF_PORT, 8580)

    device_info = DeviceInfo(
        identifiers={(DOMAIN, f"{host}:{port}")},
        name="Lionel MTH Bridge",
        manufacturer="Allen Nemetz",
        model="O-Gauge Command Control Bridge",
        sw_version=SW_VERSION,
    )

    async_add_entities([RefreshEnginesButton(coordinator, device_info)])


class RefreshEnginesButton(CoordinatorEntity, ButtonEntity):
    """Button to trigger engine library rescan on the bridge."""

    def __init__(
        self,
        coordinator: LionelMthBridgeCoordinator,
        device_info: DeviceInfo,
    ) -> None:
        super().__init__(coordinator)
        self._attr_unique_id = (
            f"{coordinator.host}:{coordinator.port}_refresh_engines"
        )
        self._attr_name = "Refresh Engines"
        self._attr_device_info = device_info
        self._attr_icon = "mdi:refresh"

    async def async_press(self) -> None:
        """Handle the button press — send POST /refresh to the bridge."""
        _LOGGER.info("Refresh Engines button pressed — triggering bridge rescan")
        ok = await self.coordinator.async_refresh_engines()
        if not ok:
            _LOGGER.warning("Failed to trigger engine refresh on bridge")
