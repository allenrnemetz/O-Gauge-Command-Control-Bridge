# pylint: disable=import-error
"""Text platform for the O Gauge Command Control Bridge integration.

Provides an editable text entity for each engine so the user can rename
engines directly from the Home Assistant UI. The new name is sent to the
bridge via POST /rename and saved in the bridge config file.
"""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.text import TextEntity, TextMode
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant, callback
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
    """Set up O Gauge Command Control Bridge text entities from a config entry."""
    coordinator: LionelMthBridgeCoordinator = hass.data[DOMAIN][entry.entry_id]
    host = entry.data[CONF_HOST]
    port = entry.data.get(CONF_PORT, 8580)

    device_info = DeviceInfo(
        identifiers={(DOMAIN, f"{host}:{port}")},
        name="O Gauge Command Control Bridge",
        manufacturer="Allen Nemetz",
        model="O-Gauge Command Control Bridge",
        sw_version=SW_VERSION,
    )

    # Track which engine IDs we've already created text entities for
    coordinator._known_text_lionel_ids = set()
    coordinator._known_text_mth_ids = set()
    coordinator._text_device_info = device_info
    coordinator._async_add_text_entities = async_add_entities

    # Add initial text entities
    _async_add_engine_text(coordinator)

    # Register a listener so new engines get text entities as they're discovered
    coordinator.async_add_listener(
        lambda: _async_add_engine_text(coordinator)
    )


@callback
def _async_add_engine_text(
    coordinator: LionelMthBridgeCoordinator,
) -> None:
    """Add text entities for any engines not yet tracked."""
    data = coordinator.data
    if not data:
        return

    device_info = coordinator._text_device_info
    entities: list[TextEntity] = []

    for eng in data.get("lionel_engines", []):
        tmcc_id = eng.get("tmcc_id")
        if tmcc_id is not None and tmcc_id not in coordinator._known_text_lionel_ids:
            coordinator._known_text_lionel_ids.add(tmcc_id)
            entities.append(
                EngineNameText(coordinator, device_info, "lionel", tmcc_id)
            )

    for eng in data.get("mth_engines", []):
        dcs_id = eng.get("dcs_id")
        if dcs_id is not None and dcs_id not in coordinator._known_text_mth_ids:
            coordinator._known_text_mth_ids.add(dcs_id)
            entities.append(
                EngineNameText(coordinator, device_info, "mth", dcs_id)
            )

    if entities:
        coordinator._async_add_text_entities(entities)


class EngineNameText(CoordinatorEntity, TextEntity):
    """Editable text entity for an engine's display name.

    Shows the current name (override if set, otherwise polled name).
    When the user edits it, the new name is sent to the bridge via
    POST /rename and saved in the bridge config file.
    Clearing the text reverts to the polled name from the controller board.
    """

    def __init__(
        self,
        coordinator: LionelMthBridgeCoordinator,
        device_info: DeviceInfo,
        engine_type: str,
        engine_id: int,
    ) -> None:
        super().__init__(coordinator)
        self._engine_type = engine_type
        self._engine_id = engine_id
        self._attr_unique_id = (
            f"{coordinator.host}:{coordinator.port}_{engine_type}_name_{engine_id}"
        )
        label = "Lionel" if engine_type == "lionel" else "MTH"
        self._attr_name = f"{label} Engine #{engine_id} Name"
        self._attr_device_info = device_info
        self._attr_icon = "mdi:pencil"
        self._attr_mode = TextMode.TEXT
        self._attr_max_length = 50

    def _find_engine(self) -> dict[str, Any] | None:
        data = self.coordinator.data
        if not data:
            return None
        key = "lionel_engines" if self._engine_type == "lionel" else "mth_engines"
        id_field = "tmcc_id" if self._engine_type == "lionel" else "dcs_id"
        for eng in data.get(key, []):
            if eng.get(id_field) == self._engine_id:
                return eng
        return None

    @property
    def native_value(self) -> str | None:
        eng = self._find_engine()
        if not eng:
            return None
        # Show the display name (override if set, otherwise polled)
        name_field = "road_name" if self._engine_type == "lionel" else "name"
        return eng.get(name_field, "")

    async def async_set_value(self, value: str) -> None:
        """Called when the user edits the text field in HA."""
        value = value.strip()
        _LOGGER.info(
            "Renaming %s engine #%d to '%s'",
            self._engine_type,
            self._engine_id,
            value or "(polled name)",
        )
        ok = await self.coordinator.async_set_engine_name(
            self._engine_type, self._engine_id, value
        )
        if not ok:
            _LOGGER.warning("Failed to set engine name on bridge")
