from collections.abc import Callable
from enum import Enum
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
from i10_bluesky.plans.utils.motors import MotorTable


class StatPosition(tuple, Enum):
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


def scan_and_move_to_fit_pos(funcs: TCallable) -> TCallable:
    """Wrapper to add PeakStats call back before performing scan
    and move to the fitted position after scan.

    Parameters
    ----------
    det: StandardReadable,
        Detector to be use for alignment.
    motor: Motor
        Motor devices that is being centre.
    fitted_loc: StatPosition | None = None,
        Which fitted position to move to see StatPosition
    detname_suffix: Str
        Name of the fitted axis within the detector
    args:
        Other arguments the wrapped scan need to function.
    kwargs:
        Other keyword arguments the wrapped scan need to function.
    """

    def inner(
        det: StandardReadable,
        motor: Motor,
        fitted_loc: StatPosition,
        detname_suffix: str,
        *args,
        **kwargs,
    ):
        ps = PeakStats(
            f"{motor.name}-user_readback",
            f"{det.name}-{detname_suffix}",
            calc_derivative_and_stats=True,
        )
        yield from bpp.subs_wrapper(
            funcs(det, motor, fitted_loc, detname_suffix, *args, **kwargs),
            ps,
        )
        peak_position = get_stat_loc(ps, fitted_loc)

        LOGGER.info(f"Fit info {ps}")
        yield from abs_set(motor, peak_position, wait=True)

    return cast(TCallable, inner)


@scan_and_move_to_fit_pos
def step_scan_and_move_fit(
    det: StandardReadable,
    motor: Motor,
    fitted_loc: StatPosition,
    detname_suffix: str,
    start: float,
    end: float,
    num: int,
) -> MsgGenerator:
    """Does a step scan and move to the fitted position
    Parameters
    ----------
    det: StandardReadable,
        Detector to be use for alignment.
    motor: Motor
        Motor devices that is being centre.
    fitted_loc: StatPosition | None = None,
        Which fitted position to move to see StatPosition
    detname_suffix: Str
        Name of the fitted axis within the detector
    start: float,
        Starting position for the scan.
    end: float,
        Ending position for the scan.
    num:int
        Number of step.
    """
    LOGGER.info(
        f"Step scanning {motor.name} with {det.name}-{detname_suffix}\
            pro-scan move to {fitted_loc}"
    )
    return scan([det], motor, start, end, num=num)


@scan_and_move_to_fit_pos
def fast_scan_and_move_fit(
    det: StandardReadable,
    motor: Motor,
    fitted_loc: StatPosition,
    detname_suffix: str,
    start: float,
    end: float,
    motor_speed: float | None = None,
) -> MsgGenerator:
    """Does a fast non-stopping scan and move to the fitted position
    Parameters
    ----------
    det: StandardReadable,
        Detector to be use for alignment.
    motor: Motor
        Motor devices that is being centre.
    fitted_loc: StatPosition
        Which fitted position to move to see StatPosition.
    detname_suffix: Str
        Name of the fitted axis within the detector
    start: float,
        Starting position for the scan.
    end: float,
        Ending position for the scan.
    motor_speed: float | None = None,
        Speed of the motor.
    """
    LOGGER.info(
        f"Fast scanning {motor.hints} with {det.name}-{detname_suffix}\
              pro-scan move to {fitted_loc}"
    )
    return fast_scan_1d(
        dets=[det], motor=motor, start=start, end=end, motor_speed=motor_speed
    )


def get_stat_loc(ps: PeakStats, loc: StatPosition) -> float:
    """Helper to check the fit was done correctly and
    return requested stats position."""
    peak_stat = ps[loc.value[0]]
    if not peak_stat:
        raise ValueError("Fitting failed, check devices name are correct.")
    peak_stat = peak_stat._asdict()

    if not peak_stat["fwhm"]:
        raise ValueError("Fitting failed, no peak within scan range.")

    stat_pos = peak_stat[loc.value[1]]
    return stat_pos if isinstance(stat_pos, float) else stat_pos[0]


def align_slit_with_look_up(
    motor: Motor,
    size: float,
    slit_table: dict[str, float],
    det: StandardReadable,
    centre_type: StatPosition,
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
    centre_type: StatPosition
        Which fitted position to move to see StatPosition.
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
        detname_suffix="value",
        end=end_pos,
        fitted_loc=centre_type,
        num=num,
    )
    temp = yield from read(motor.user_readback)
    slit_table[str(size)] = temp[motor.name + "-user_readback"]["value"]
