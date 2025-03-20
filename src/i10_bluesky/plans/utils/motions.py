from collections.abc import Hashable

import bluesky.plan_stubs as bps
from bluesky.plan_stubs import abs_set
from dodal.common.types import MsgGenerator
from dodal.devices.slits import Slits
from ophyd_async.epics.motor import Motor
from pydantic import RootModel

from i10_bluesky.log import LOGGER


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


def set_slit_size(
    xy_slit: Slits,
    x_size: float,
    y_size: float | None = None,
    wait: bool = True,
    group: Hashable | None = None,
) -> MsgGenerator:
    """Set opening of an x-y slit.

    Parameters
    ----------
    xy_slit: Slits
        A slits device.
    x_size: float
        The x opening size.
    y_size: float
        The y opening size.
    wait: bool
        If this is true it will wait for all motions to finish.
    group: Hashable
        If given this will be the group name that pass along to bluesky, which
        can be use at a later time.
    """

    if wait and group is None:
        group = f"{xy_slit.name}_wait"
    if y_size is None:
        y_size = x_size
    LOGGER.info(f"Setting {xy_slit.name} to x = {x_size}, y = {y_size}.")
    yield from bps.abs_set(xy_slit.x_gap, x_size, group=group)
    yield from bps.abs_set(xy_slit.y_gap, y_size, group=group)
    if wait:
        LOGGER.info(f"Waiting for {xy_slit.name} to finish move.")
        yield from bps.wait(group=group)
