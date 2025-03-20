from collections.abc import Hashable

import bluesky.plan_stubs as bps
from dodal.beamlines.i10 import (
    diffractometer,
    simple_stage,
)
from dodal.common.types import MsgGenerator
from ophyd_async.core import StandardReadable

from i10_bluesky.plans.configuration.default_setting import (
    RASOR_DEFAULT_DET,
    RASOR_DEFAULT_DET_NAME_EXTENSION,
)
from i10_bluesky.plans.utils.alignments import StatPosition, step_scan_and_move_fit


def centre_tth(
    det: StandardReadable = RASOR_DEFAULT_DET,
    det_name: str = RASOR_DEFAULT_DET_NAME_EXTENSION,
    start: float = -1,
    end: float = 1,
    num: int = 21,
) -> MsgGenerator:
    """Centre two theta using Rasor dector."""

    yield from step_scan_and_move_fit(
        det=det,
        motor=diffractometer().tth,
        start=start,
        end=end,
        num=num,
        detname_suffix=det_name,
        fitted_loc=StatPosition.CEN,
    )


def centre_alpha(
    det: StandardReadable = RASOR_DEFAULT_DET,
    det_name: str = RASOR_DEFAULT_DET_NAME_EXTENSION,
    start: float = -0.8,
    end: float = 0.8,
    num: int = 21,
) -> MsgGenerator:
    """Centre rasor alpha using Rasor dector."""
    yield from step_scan_and_move_fit(
        det=det,
        motor=diffractometer().alpha,
        start=start,
        end=end,
        num=num,
        detname_suffix=det_name,
        fitted_loc=StatPosition.CEN,
    )


def centre_det_angles(
    det: StandardReadable = RASOR_DEFAULT_DET,
    det_name: str = RASOR_DEFAULT_DET_NAME_EXTENSION,
) -> MsgGenerator:
    """Centre both two theta and alpha angle on Rasor"""
    yield from centre_tth(det, det_name)
    yield from centre_alpha(det, det_name)


def move_pin_origin(wait: bool = True, group: Hashable | None = None) -> MsgGenerator:
    """Move the point to the centre of rotation."""

    if wait and group is None:
        group = "move_pin_origin"
    yield from bps.abs_set(simple_stage().x, 0, wait=False, group=group)
    yield from bps.abs_set(simple_stage().y, 0, wait=False, group=group)
    yield from bps.abs_set(simple_stage().z, 0, wait=False, group=group)
    if wait:
        yield from bps.wait(group=group)
