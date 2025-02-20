from collections.abc import Callable
from enum import Enum
from functools import wraps
from typing import TypeVar, cast

from bluesky import preprocessors as bpp
from bluesky.callbacks.fitting import PeakStats
from bluesky.plan_stubs import abs_set, read
from bluesky.plans import scan
from dodal.common.types import MsgGenerator
from ophyd_async.core import StandardReadable
from ophyd_async.epics.motor import Motor
from p99_bluesky.plans.fast_scan import fast_scan_1d

from i10_bluesky.log import LOGGER
from i10_bluesky.plans.utils.helpers import cal_range_num
from i10_bluesky.plans.utils.motions import MotorTable


class PeakPosition(tuple, Enum):
    """Data table to help access the fit data.\n
    Com: Centre of mass\n
    CEN: Peak position\n
    MIN: Minimum value\n
    MAX: Maximum value\n
    With prefix D: Differential
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


def scan_and_move_cen(funcs: TCallable) -> TCallable:
    """Wrapper to add PeakStats call back before performing scan
    and move to the fitted position after scan."""

    @wraps(funcs)
    def inner(**kwargs):
        # Add default value if they are None or missing.
        if "motor_name" not in kwargs or kwargs["motor_name"] is None:
            kwargs["motor_name"] = "-user_readback"
        if "det_name" not in kwargs or kwargs["det_name"] is None:
            kwargs["det_name"] = "-value"
        if "loc" not in kwargs or kwargs["loc"] is None:
            kwargs["loc"] = PeakPosition.CEN
        ps = PeakStats(
            f"{kwargs['motor'].name}{kwargs['motor_name']}",
            f"{kwargs['det'].name}{kwargs['det_name']}",
            calc_derivative_and_stats=True,
        )
        yield from bpp.subs_wrapper(
            funcs(**kwargs),
            ps,
        )
        peak_position = get_stat_loc(ps, kwargs["loc"])

        LOGGER.info(f"Fit info {ps}")
        yield from abs_set(kwargs["motor"], peak_position, wait=True)

    return cast(TCallable, inner)


@scan_and_move_cen
def step_scan_and_move_cen(
    det: StandardReadable,
    motor: Motor,
    start: float,
    end: float,
    num: int,
    motor_name: str | None = None,
    det_name: str | None = None,
    loc: PeakPosition | None = None,
) -> MsgGenerator:
    """Does a step scan and move to the fitted position.

    Parameters
    ----------
    det: StandardReadable
        Detector to be use for alignment.
    motor: Motor
        Motor devices that is being centre.
    start: float
        Starting position for the scan.
    end: float
        Ending position for the scan.
    num:int
        Number of step.
    motor_name: str | None
        Name extension for the motor.
    det_name: str | None
        Name extension for the det.
    loc: PeakPosition | None
        Which fitted position to move to see PeakPosition.
    """
    LOGGER.info(
        f"Step scanning {motor}{motor_name} with {det}{det_name} pro-scan move to {loc}"
    )
    return scan([det], motor, start, end, num=num)


@scan_and_move_cen
def fast_scan_and_move_cen(
    det: StandardReadable,
    motor: Motor,
    start: float,
    end: float,
    motor_name: str,
    det_name: str | None = None,
    loc: PeakPosition | None = None,
    motor_speed: float | None = None,
) -> MsgGenerator:
    """Does a fast non-stopping scan and move to the fitted position.

    Parameters
    ----------
     det: StandardReadable,
         Detector to be use for alignment.
     motor: Motor
         Motor devices that is being centre.
     start: float
         Starting position for the scan.
     end: float
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
    LOGGER.info(
        f"Fast scaning {motor}{motor_name} with {det}{det_name} pro-scan move to {loc}"
    )
    return fast_scan_1d([det], motor, start, end, motor_speed=motor_speed)


def get_stat_loc(ps: PeakStats, loc: PeakPosition) -> float:
    """Helper to check the fit was done correctly and
    return requested stats position (peak position)."""
    stat = getattr(ps, loc.value[0])
    if not stat:
        raise ValueError("Fitting failed, check devices name are correct.")
    elif not stat.fwhm:
        raise ValueError("Fitting failed, no peak within scan range.")

    stat_pos = getattr(stat, loc.value[1])
    return stat_pos if isinstance(stat_pos, float) else stat_pos[0]


def align_slit_with_look_up(
    motor: Motor,
    size: float,
    slit_table: dict[str, float],
    det: StandardReadable,
    det_name: str | None = None,
    motor_name: str | None = None,
    centre_type: PeakPosition | None = None,
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
    yield from step_scan_and_move_cen(
        det=det,
        motor=motor,
        start=start_pos,
        end=end_pos,
        num=num,
        det_name=det_name,
        motor_name=motor_name,
        loc=centre_type,
    )
    temp = yield from read(motor.user_readback)
    slit_table[str(size)] = temp[motor.name + "-user_readback"]["value"]
