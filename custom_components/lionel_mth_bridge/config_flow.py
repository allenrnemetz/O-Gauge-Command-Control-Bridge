"""Config flow for the O Gauge Command Control Bridge integration."""

from __future__ import annotations

import voluptuous as vol
from homeassistant import config_entries
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant
from homeassistant.data_entry_flow import FlowResult
import aiohttp

from .const import DEFAULT_PORT, DOMAIN, STATUS_ENDPOINT

STEP_USER_DATA_SCHEMA = vol.Schema(
    {
        vol.Required(CONF_HOST): str,
        vol.Optional(CONF_PORT, default=DEFAULT_PORT): int,
    }
)


async def _test_connection(hass: HomeAssistant, host: str, port: int) -> bool:
    """Test if the bridge is reachable and returns valid status JSON."""
    url = f"http://{host}:{port}{STATUS_ENDPOINT}"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url, timeout=aiohttp.ClientTimeout(total=10)) as resp:
                if resp.status == 200:
                    data = await resp.json()
                    return "online" in data
    except Exception:
        pass
    return False


class LionelMthBridgeConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Handle a config flow for O Gauge Command Control Bridge."""

    VERSION = 1

    async def async_step_user(
        self, user_input: dict[str, any] | None = None
    ) -> FlowResult:
        """Handle the initial step."""
        if user_input is not None:
            host = user_input[CONF_HOST]
            port = user_input[CONF_PORT]

            # Test connection
            ok = await _test_connection(self.hass, host, port)
            if not ok:
                return self.async_show_form(
                    step_id="user",
                    data_schema=STEP_USER_DATA_SCHEMA,
                    errors={"base": "cannot_connect"},
                    description_placeholders={"host": host, "port": str(port)},
                )

            # Check for duplicate
            await self.async_set_unique_id(f"{host}:{port}")
            self._abort_if_unique_id_configured()

            return self.async_create_entry(
                title=f"O Gauge Command Control Bridge ({host})",
                data=user_input,
            )

        return self.async_show_form(
            step_id="user",
            data_schema=STEP_USER_DATA_SCHEMA,
        )
