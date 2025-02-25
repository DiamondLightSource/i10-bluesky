from collections.abc import Hashable

from bluesky.plan_stubs import abs_set
from dodal.common.types import MsgGenerator
from ophyd_async.epics.motor import Motor
from pydantic import RootModel


class MotorTable(RootModel):
    root: dict[str, float]


def move_motor_with_look_up(
    slit: Motor,
    size: float,
    motor_table: dict[str, float],
    use_motor_position: bool = False,
    wait: bool = True,
    group: Hashable | None = None,
) -> MsgGenerator:
    """Perform a step scan with the the range and starting motor position
      given/calculated by using a look up table(dictionary).
      Move to the peak position after the scan and update the lookup table.
    Parameters
    ----------
    motor: Motor
        Motor devices that is being centre.
    size: float,
        The size/name in the motor_table.
    slit_table: dict[str, float],
        Look up table for motor position, the str part should be the size of
        the slit in um.
    det: StandardReadable,
        Detector to be use for alignment.
    det_name: str | None = None,
        Name extension for the det.
    motor_name: str | None = None,
        Name extension for the motor.
    centre_type: StatPosition | None = None,
        Which fitted position to move to see StatPosition.
    """
    MotorTable.model_validate(motor_table)
    if use_motor_position:
        yield from abs_set(slit, size, wait=wait, group=group)
    elif str(int(size)) in motor_table:
        yield from abs_set(slit, motor_table[str(int(size))], wait=wait, group=group)
    else:
        raise ValueError(
            f"No slit with size={size}. Available slit size: {motor_table}"
        )
