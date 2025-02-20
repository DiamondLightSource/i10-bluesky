from ophyd_async.core import (
    StandardReadable,
)
from ophyd_async.core import StandardReadableFormat as Format
from ophyd_async.epics.core import epics_signal_rw


class sim_detector(StandardReadable):
    def __init__(self, prefix: str, name: str):
        with self.add_children_as_readables(Format.HINTED_SIGNAL):
            self.value = epics_signal_rw(float, read_pv=prefix)
        super().__init__(name=name)
