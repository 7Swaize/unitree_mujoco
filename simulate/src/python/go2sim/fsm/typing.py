from typing import Any

import iceoryx2 as iox2
from iceoryx_interfaces.mappings import CommandKind, SportCommand

_CommandItem = tuple[CommandKind, SportCommand, list[Any], iox2.ActiveRequest]