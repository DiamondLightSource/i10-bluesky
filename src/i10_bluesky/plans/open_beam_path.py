from collections.abc import Hashable

import bluesky.plan_stubs as bps
from dodal.beamlines.i10 import det_slits, pa_stage, pin_hole, slits
from dodal.common.types import MsgGenerator

from i10_bluesky.log import LOGGER
from i10_bluesky.plans.configuration.default_setting import (
    DSD_DSU_OPENING_POS,
    PIN_HOLE_OPEING_POS,
    S5S6_OPENING_SIZE,
)
from i10_bluesky.plans.utils import set_slit_size


def open_s5s6(
    size: float = S5S6_OPENING_SIZE, wait: bool = True, group: Hashable | None = None
) -> MsgGenerator:
    """Open up clean up slits s5 and s6.

    Parameters
    ----------
    size: float
        The size of their opening both x and y will set to the same value.
    wait: bool
        If this is true it will set both slits to move and wait until it get to position
        .
    group: Hashable
        If given this will be the group name that pass along to bluesky, which
        can be use at a later time.
    """

    if wait and group is None:
        group = f"{slits().name}__wait"
    yield from set_slit_size(slits().s5, size, wait=False, group=group)
    yield from set_slit_size(slits().s6, size, wait=False, group=group)
    if wait:
        LOGGER.info("Waiting for s5 and s6 to finish move.")
        yield from bps.wait(group=group)


def open_dsd_dsu(
    open_position: float = DSD_DSU_OPENING_POS,
    wait: bool = True,
    group: Hashable | None = None,
) -> MsgGenerator:
    """Remove both detector slits
    Parameters
    ----------
    open_position: float
        The position of the opening.
    wait: bool
        If this is true it will wait until all motions are done.
    group: Hashable
        If given this will be the group name that pass along to bluesky, which
        can be use at a later time.
    """

    if wait and group is None:
        group = f"{det_slits().name}_wait"
    LOGGER.info("Opening Dsd and dsu, very slow motor may take minutes.")
    yield from bps.abs_set(det_slits().upstream, open_position, group=group)
    yield from bps.abs_set(det_slits().downstream, open_position, group=group)
    if wait:
        LOGGER.info("Waiting for dsd and dsu to finish move.")
        yield from bps.wait(group=group)


def remove_pin_hole(
    open_position: float = PIN_HOLE_OPEING_POS,
    wait: bool = True,
    group: Hashable | None = None,
) -> MsgGenerator:
    """Move pin hole out of the way of the pin
    Parameters
    ----------
    open_position: float
        The position of the opening.
    wait: bool
        If this is true it will wait until all motions are done.
    group: Hashable
        If given this will be the group name that pass along to bluesky, which
        can be use at a later time.
    """
    if wait and group is None:
        group = f"{pin_hole().name}_wait"
    LOGGER.info("Removing pin hole.")
    yield from bps.abs_set(pin_hole().x, open_position, wait=False, group=group)
    if wait:
        LOGGER.info(f"Waiting for {pin_hole().name} to finish move.")
        yield from bps.wait(group=group)


def direct_beam_polan(wait: bool = True, group: Hashable | None = None) -> MsgGenerator:
    """
    Move polarization analyzer out of the way of the beam.
    Parameters
    ----------
    wait: bool
        If this is true it will wait until all motions are done.
    group: Hashable
        If given this will be the group name that pass along to bluesky, which
        can be use at a later time.
    """
    if wait and group is None:
        group = f"{pa_stage().name}_wait"
    LOGGER.info(f"Removing {pa_stage().name}.")
    yield from bps.abs_set(pa_stage().eta, 0, wait=False, group=group)
    yield from bps.abs_set(pa_stage().py, 0, wait=False, group=group)
    yield from bps.abs_set(pa_stage().ttp, 0, wait=False, group=group)
    yield from bps.abs_set(pa_stage().thp, 0, wait=False, group=group)
    if wait:
        LOGGER.info(f"Waiting for {pa_stage().name} to finish move.")
        yield from bps.wait(group=group)


def clear_beam_path(wait: bool = True, group: Hashable | None = None) -> MsgGenerator:
    """Move everything out of the way of the direct beam
    Parameters
    ----------
    wait: bool
        If this is true it will wait until all motions are done.
    group: Hashable
        If given this will be the group name that pass along to bluesky, which
        can be use at a later time.

    """
    if group is None:
        group = "clear_beam_path"
    yield from open_s5s6(wait=False, group=group)
    yield from open_dsd_dsu(wait=False, group=group)
    yield from remove_pin_hole(wait=False, group=group)
    yield from direct_beam_polan(wait=False, group=group)
    if wait:
        yield from bps.wait(group=group)
