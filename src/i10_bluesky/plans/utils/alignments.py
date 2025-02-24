from collections.abc import Callable
from enum import Enum
from typing import TypeVar, cast

from bluesky import preprocessors as bpp
from bluesky.callbacks.fitting import PeakStats
from bluesky.plan_stubs import abs_set, read
from bluesky.plans import scan
from bluesky.protocols import Movable
from dodal.common.types import MsgGenerator
from ophyd_async.core import Device, StandardReadable
from ophyd_async.epics.motor import Motor
from p99_bluesky.plans.fast_scan import fast_scan_1d

from i10_bluesky.log import LOGGER
from i10_bluesky.plans.utils.helpers import cal_range_num
from i10_bluesky.plans.utils.motors import MotorTable


class PeakPosition(tuple, Enum):
    """
    Data table to help access the fit data.
    Com: Centre of mass
    CEN: Peak position
    MIN: Minimum value
    MAX: Maximum value
    D_: Differential
    """

    COM = ("stats", "com")
    CEN = ("stats", "cen")
    MIN = ("stats", "min")
    MAX = ("stats", "max")
    D_COM = ("derivative_stats", "com")
    D_CEN = ("derivative_stats", "cen")
    D_MIN = ("derivative_stats", "min")
    D_MAX = ("derivative_stats", "max")


TCallable = TypeVar("TCallable", bound=Callable)


class MoveableDevice(Device, Movable): ...


def scan_and_move_to_fit_pos(funcs: TCallable) -> TCallable:
    """Wrapper to add PeakStats call back before performing scan
    and move to the fitted position after scan"""

    def inner(
        det: StandardReadable,
        motor: MoveableDevice,
        start: float,
        end: float,
        loc: PeakPosition,
        **kwarg,
    ):
        ps = PeakStats(
            f"{motor.name}-user_readback",
            f"{det.name}-value",
            calc_derivative_and_stats=True,
        )
        yield from bpp.subs_wrapper(
            funcs(det, motor, start, end, loc, **kwarg),
            ps,
        )
        peak_position = get_stat_loc(ps, loc)

        LOGGER.info(f"Fit info {ps}")
        yield from abs_set(motor, peak_position, wait=True)

    return cast(TCallable, inner)


@scan_and_move_to_fit_pos
def step_scan_and_move_fit(
    det: StandardReadable,
    motor: Motor,
    start: float,
    end: float,
    loc: PeakPosition,
    num: int,
) -> MsgGenerator:
    """Does a step scan and move to the fitted position
       Parameters
    ----------
    det: StandardReadable,
        Detector to be use for alignment.
    motor: Motor
        Motor devices that is being centre.
    start: float,
        Starting position for the scan.
    end: float,
        Ending position for the scan.
    num:int
        Number of step.
    motor_name: str | None = None,
        Name extension for the motor.
    det_name: str | None = None,
        Name extension for the det.
    loc: PeakPosition | None = None,
        Which fitted position to move to see PeakPosition
    """
    LOGGER.info(f"Step scanning {motor.name} with {det.name} pro-scan move to {loc}")
    return scan([det], motor, start, end, num=num)


@scan_and_move_to_fit_pos
def fast_scan_and_move_fit(
    det: StandardReadable,
    motor: Motor,
    start: float,
    end: float,
    loc: PeakPosition,
    motor_speed: float | None = None,
) -> MsgGenerator:
    """Does a fast non-stopping scan and move to the fitted position
    Parameters
    ----------
    det: StandardReadable,
        Detector to be use for alignment.
    motor: Motor
        Motor devices that is being centre.
    start: float,
        Starting position for the scan.
    end: float,
        Ending position for the scan.
    det_name: str | None = None,
        Name extension for the det.
    motor_name: str | None = None,
        Name extension for the motor.
    loc: PeakPosition | None = None,
        Which fitted position to move to see PeakPosition.
    motor_speed: float | None = None,
        Speed of the motor.
    """
    LOGGER.info(f"Fast scaning {motor.hints} with {det.hints} pro-scan move to {loc}")
    return fast_scan_1d([det], motor, start, end, motor_speed=motor_speed)


def get_stat_loc(ps: PeakStats, loc: PeakPosition) -> float:
    """Helper to check the fit was done correctly and
    return requested stats position (peak position)."""
    peak_stat = ps[loc.value[0]]
    if not peak_stat:
        raise ValueError("Fitting failed, check devices name are correct.")
    stat = ps[loc.value[0]]._asdict()
    print(stat)
    if not stat["fwhm"]:
        raise ValueError("Fitting failed, no peak within scan range.")

    stat_pos = stat[loc.value[1]]
    return stat_pos if isinstance(stat_pos, float) else stat_pos[0]


def align_slit_with_look_up(
    motor: Motor,
    size: float,
    slit_table: dict[str, float],
    det: StandardReadable,
    centre_type: PeakPosition,
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
    motor_table: dict[str, float],
        Look up table for motor position, the str part should be the size of
        the slit in um.
    det: StandardReadable,
        Detector to be use for alignment.
    det_name: str | None = None,
        Name extension for the det.
    motor_name: str | None = None,
        Name extension for the motor.
    centre_type: PeakPosition | None = None,
        Which fitted position to move to see PeakPosition.
    """
    MotorTable.model_validate(slit_table)
    if str(int(size)) in slit_table:
        start_pos, end_pos, num = cal_range_num(
            cen=slit_table[str(size)], range=size / 1000 * 3, size=size / 5000.0
        )
    else:
        raise ValueError(f"Size of {size} is not in {slit_table.keys}")
    yield from step_scan_and_move_fit(
        det=det,
        motor=motor,
        start=start_pos,
        end=end_pos,
        loc=centre_type,
        num=num,
    )
    temp = yield from read(motor.user_readback)
    slit_table[str(size)] = temp[motor.name + "-user_readback"]["value"]
