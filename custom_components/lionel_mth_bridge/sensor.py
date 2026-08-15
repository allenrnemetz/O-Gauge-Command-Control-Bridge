"""Sensor platform for the Lionel MTH Bridge integration."""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.binary_sensor import (
    BinarySensorEntity,
    BinarySensorDeviceClass,
)
from homeassistant.components.sensor import SensorEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.entity import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import LionelMthBridgeCoordinator
from .const import DOMAIN, SW_VERSION

_LOGGER = logging.getLogger(__name__)

# Loco type mapping (from PyTrain EngineType enum)
LOCO_TYPES = {
    0: "Diesel",
    1: "Steam",
    2: "Electric",
    3: "Subway",
    4: "Accessory",
    5: "Passenger Car",
    6: "Breakdown",
    7: "Reserved",
    8: "Acela",
    9: "Crane",
    10: "Diesel Switcher",
    11: "Steam Switcher",
    12: "Freight Sounds",
    13: "Diesel Pullmor",
    14: "Steam Pullmor",
    15: "Transformer",
}


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up Lionel MTH Bridge sensors from a config entry."""
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

    entities: list[SensorEntity | BinarySensorEntity] = [
        # Binary sensors — connection status
        BridgeOnlineBinarySensor(coordinator, device_info),
        Base3ConnectedBinarySensor(coordinator, device_info),
        WtiuConnectedBinarySensor(coordinator, device_info),
        # Sensors — status info
        WtiuHostSensor(coordinator, device_info),
        LionelEngineCountSensor(coordinator, device_info),
        MthEngineCountSensor(coordinator, device_info),
    ]

    async_add_entities(entities)

    # Track which engine IDs we've already created sensors for
    coordinator._known_lionel_ids = set()
    coordinator._known_mth_ids = set()
    coordinator._engine_device_info = device_info
    coordinator._async_add_engine_entities = async_add_entities
    coordinator._engine_entity_registry = None

    # Add initial engine sensors
    _async_add_engine_sensors(coordinator)

    # Register a listener so new engines get sensors and stale ones are removed
    coordinator.async_add_listener(
        lambda: _async_update_engine_sensors(coordinator, hass, entry)
    )


@callback
def _async_add_engine_sensors(
    coordinator: LionelMthBridgeCoordinator,
) -> None:
    """Add per-engine sensors for any engines not yet tracked.

    Called on initial setup and on every coordinator update.
    Only adds sensors for new engine IDs — existing sensors update
    automatically via the coordinator.
    """
    data = coordinator.data
    if not data:
        return

    device_info = coordinator._engine_device_info
    entities: list[SensorEntity] = []

    for eng in data.get("lionel_engines", []):
        tmcc_id = eng.get("tmcc_id")
        if tmcc_id is not None and tmcc_id not in coordinator._known_lionel_ids:
            coordinator._known_lionel_ids.add(tmcc_id)
            entities.append(
                LionelEngineSensor(coordinator, device_info, tmcc_id)
            )

    for eng in data.get("mth_engines", []):
        dcs_id = eng.get("dcs_id")
        if dcs_id is not None and dcs_id not in coordinator._known_mth_ids:
            coordinator._known_mth_ids.add(dcs_id)
            # Look up Lionel TMCC address from discovered mappings
            lionel_id = None
            discovered = data.get("engine_mappings", {}).get("discovered", {})
            for lionel_addr, mth_dcs in discovered.items():
                if mth_dcs == dcs_id:
                    lionel_id = int(lionel_addr)
                    break
            entities.append(
                MthEngineSensor(coordinator, device_info, dcs_id, lionel_id)
            )

    if entities:
        coordinator._async_add_engine_entities(entities)


@callback
def _async_update_engine_sensors(
    coordinator: LionelMthBridgeCoordinator,
    hass: HomeAssistant,
    entry: ConfigEntry,
) -> None:
    """Add new engine sensors and remove sensors for engines no longer present.

    Called on every coordinator update. Adds sensors for newly discovered
    engines and removes entities for engines that are no longer reported
    by the bridge.
    """
    data = coordinator.data
    if not data:
        return

    # First, add any new engine sensors
    _async_add_engine_sensors(coordinator)

    # Build set of current Lionel engine IDs from bridge data
    current_lionel_ids = {
        eng["tmcc_id"] for eng in data.get("lionel_engines", [])
        if eng.get("tmcc_id") is not None
    }

    # Find stale Lionel IDs (we created sensors for them before, but Base 3 no longer reports them)
    # Only Lionel entries are cleaned up — HA should always match the Base 3 library.
    # MTH entries are kept even when stale so the user can rename them and maintain
    # the library even when engines aren't on the track.
    stale_lionel = coordinator._known_lionel_ids - current_lionel_ids

    if not stale_lionel:
        return

    # Remove stale Lionel entities via the entity registry
    from homeassistant.helpers.entity_registry import async_get as async_get_entity_registry

    entity_registry = async_get_entity_registry(hass)
    host = coordinator.host
    port = coordinator.port

    for tmcc_id in stale_lionel:
        unique_id = f"{host}:{port}_lionel_engine_{tmcc_id}"
        entity_id = entity_registry.async_get_entity_id("sensor", DOMAIN, unique_id)
        if entity_id:
            entity_registry.async_remove(entity_id)
            _LOGGER.info("Removed stale Lionel engine sensor #%d (entity: %s)", tmcc_id, entity_id)
        coordinator._known_lionel_ids.discard(tmcc_id)


class BridgeBinarySensorBase(CoordinatorEntity, BinarySensorEntity):
    """Base class for bridge binary sensors."""

    def __init__(
        self,
        coordinator: LionelMthBridgeCoordinator,
        device_info: DeviceInfo,
        key: str,
        name: str,
    ) -> None:
        super().__init__(coordinator)
        self._attr_unique_id = f"{coordinator.host}:{coordinator.port}_{key}"
        self._attr_name = name
        self._attr_device_info = device_info
        self._key = key

    @property
    def is_on(self) -> bool:
        data = self.coordinator.data
        if not data:
            return False
        return bool(data.get(self._key, False))


class BridgeOnlineBinarySensor(BridgeBinarySensorBase):
    """Binary sensor for bridge online status."""

    def __init__(self, coordinator, device_info):
        super().__init__(
            coordinator,
            device_info,
            "online",
            "Bridge Online",
        )
        self._attr_device_class = BinarySensorDeviceClass.CONNECTIVITY
        self._attr_icon = "mdi:bridge"


class Base3ConnectedBinarySensor(BridgeBinarySensorBase):
    """Binary sensor for Base 3 / SER2 connection."""

    def __init__(self, coordinator, device_info):
        super().__init__(
            coordinator,
            device_info,
            "base3_connected",
            "Base 3 Connected",
        )
        self._attr_device_class = BinarySensorDeviceClass.CONNECTIVITY
        self._attr_icon = "mdi:connection"


class WtiuConnectedBinarySensor(BridgeBinarySensorBase):
    """Binary sensor for WTIU connection."""

    def __init__(self, coordinator, device_info):
        super().__init__(
            coordinator,
            device_info,
            "wtiu_connected",
            "WTIU Connected",
        )
        self._attr_device_class = BinarySensorDeviceClass.CONNECTIVITY
        self._attr_icon = "mdi:wifi"


class BridgeSensorBase(CoordinatorEntity, SensorEntity):
    """Base class for bridge sensors."""

    def __init__(
        self,
        coordinator: LionelMthBridgeCoordinator,
        device_info: DeviceInfo,
        key: str,
        name: str,
    ) -> None:
        super().__init__(coordinator)
        self._attr_unique_id = f"{coordinator.host}:{coordinator.port}_{key}"
        self._attr_name = name
        self._attr_device_info = device_info
        self._key = key

    @property
    def native_value(self):
        data = self.coordinator.data
        if not data:
            return None
        return data.get(self._key)


class WtiuHostSensor(BridgeSensorBase):
    """Sensor for WTIU host address."""

    def __init__(self, coordinator, device_info):
        super().__init__(coordinator, device_info, "wtiu_host", "WTIU Host")
        self._attr_icon = "mdi:ip-network"


class LionelEngineCountSensor(BridgeSensorBase):
    """Sensor for Lionel engine count."""

    def __init__(self, coordinator, device_info):
        super().__init__(
            coordinator, device_info, "lionel_engine_count", "Lionel Engine Count"
        )
        self._attr_icon = "mdi:train"
        self._attr_native_unit_of_measurement = "engines"

    @property
    def native_value(self) -> int:
        data = self.coordinator.data
        if not data:
            return 0
        return len(data.get("lionel_engines", []))


class MthEngineCountSensor(BridgeSensorBase):
    """Sensor for MTH engine count."""

    def __init__(self, coordinator, device_info):
        super().__init__(
            coordinator, device_info, "mth_engine_count", "MTH Engine Count"
        )
        self._attr_icon = "mdi:train"
        self._attr_native_unit_of_measurement = "engines"

    @property
    def native_value(self) -> int:
        data = self.coordinator.data
        if not data:
            return 0
        return len(data.get("mth_engines", []))


class LionelEngineSensor(CoordinatorEntity, SensorEntity):
    """Sensor for a single Lionel engine from the Base 3 library."""

    def __init__(
        self,
        coordinator: LionelMthBridgeCoordinator,
        device_info: DeviceInfo,
        tmcc_id: int,
    ) -> None:
        super().__init__(coordinator)
        self._tmcc_id = tmcc_id
        self._attr_unique_id = (
            f"{coordinator.host}:{coordinator.port}_lionel_engine_{tmcc_id}"
        )
        self._attr_name = f"Lionel Engine #{tmcc_id}"
        self._attr_device_info = device_info
        self._attr_icon = "mdi:train"

    def _find_engine(self) -> dict[str, Any] | None:
        data = self.coordinator.data
        if not data:
            return None
        for eng in data.get("lionel_engines", []):
            if eng.get("tmcc_id") == self._tmcc_id:
                return eng
        return None

    @property
    def native_value(self) -> str | None:
        eng = self._find_engine()
        if not eng:
            # Engine was discovered before but is no longer in the library.
            # This happens if the engine was removed from Base 3 or the
            # bridge lost connection during the last scan.
            return None
        name = eng.get("road_name", "Unknown")
        number = eng.get("road_number")
        if number:
            return f"{name} #{number}"
        return name

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        eng = self._find_engine()
        if not eng:
            return {"tmcc_id": self._tmcc_id, "status": "not_found"}
        loco_type = eng.get("loco_type")
        attrs = {
            "road_name": eng.get("road_name"),
            "road_number": eng.get("road_number"),
            "loco_type": loco_type,
            "loco_type_name": LOCO_TYPES.get(loco_type, "Unknown"),
            "tmcc_id": self._tmcc_id,
            "last_train_id": eng.get("last_train_id"),
            "status": "active",
        }
        # Show the original polled name if an override is in use
        if eng.get("polled_name"):
            attrs["polled_name"] = eng["polled_name"]
            attrs["name_overridden"] = True
        return attrs


class MthEngineSensor(CoordinatorEntity, SensorEntity):
    """Sensor for a single MTH engine from the WTIU."""

    def __init__(
        self,
        coordinator: LionelMthBridgeCoordinator,
        device_info: DeviceInfo,
        dcs_id: int,
        lionel_id: int | None = None,
    ) -> None:
        super().__init__(coordinator)
        self._dcs_id = dcs_id
        self._lionel_id = lionel_id
        self._attr_unique_id = (
            f"{coordinator.host}:{coordinator.port}_mth_engine_{dcs_id}"
        )
        # Use Lionel TMCC address in the name if known (that's what the user
        # uses on their Cab remote), otherwise fall back to DCS ID
        if lionel_id is not None:
            self._attr_name = f"MTH Engine #{lionel_id}"
        else:
            self._attr_name = f"MTH Engine #{dcs_id}"
        self._attr_device_info = device_info
        self._attr_icon = "mdi:train"

    def _find_engine(self) -> dict[str, Any] | None:
        data = self.coordinator.data
        if not data:
            return None
        for eng in data.get("mth_engines", []):
            if eng.get("dcs_id") == self._dcs_id:
                return eng
        return None

    @property
    def native_value(self) -> str | None:
        eng = self._find_engine()
        if not eng:
            return None
        return eng.get("name", "Unknown")

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        eng = self._find_engine()
        if not eng:
            return {"dcs_id": self._dcs_id, "status": "not_found"}
        loco_type = eng.get("type", 0)
        attrs = {
            "dcs_id": self._dcs_id,
            "lionel_id": self._lionel_id,
            "name": eng.get("name"),
            "type": loco_type,
            "is_steam": eng.get("is_steam", False),
            "is_diesel": eng.get("is_diesel", False),
            "protowhistle": eng.get("protowhistle", False),
            "status": "active",
        }
        # Show the original polled name if an override is in use
        if eng.get("polled_name"):
            attrs["polled_name"] = eng["polled_name"]
            attrs["name_overridden"] = True
        return attrs
