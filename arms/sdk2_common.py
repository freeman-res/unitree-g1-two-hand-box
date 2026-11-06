from __future__ import annotations

import logging
from typing import Optional

_initialized = False
_initialized_interface: Optional[str] = None


def initialize_sdk2(network_interface: str = "") -> None:
    """Initialize Unitree SDK2 DDS once per Python process."""
    global _initialized, _initialized_interface
    if _initialized:
        logging.info("SDK2 already initialized interface=%s", _initialized_interface or "default")
        return

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    except ImportError as exc:
        raise RuntimeError(
            "Unitree SDK2 Python package is not installed. Install unitree_sdk2py on the robot first."
        ) from exc

    if network_interface:
        logging.info("SDK2 initialize interface=%s", network_interface)
        ChannelFactoryInitialize(0, network_interface)
    else:
        logging.info("SDK2 initialize default interface")
        ChannelFactoryInitialize(0)

    _initialized = True
    _initialized_interface = network_interface or "default"
