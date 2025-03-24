from collections import defaultdict
from unittest.mock import Mock, call, patch

from bluesky.run_engine import RunEngine
from bluesky.simulators import RunEngineSimulator
from dodal.beamlines.i10 import diffractometer, simple_stage

from i10_bluesky.plans import (
    centre_alpha,
    centre_det_angles,
    centre_tth,
    move_pin_origin,
)
from i10_bluesky.plans.configuration.default_setting import (
    RASOR_DEFAULT_DET,
    RASOR_DEFAULT_DET_NAME_EXTENSION,
)
from i10_bluesky.plans.utils.alignments import StatPosition

from ..helper_functions import check_msg_set, check_msg_wait

docs = defaultdict(list)


def capture_emitted(name, doc):
    docs[name].append(doc)


@patch("i10_bluesky.plans.centre_direct_beam.step_scan_and_move_fit")
async def test_centre_tth(
    fake_step_scan_and_move_fit: Mock,
    RE: RunEngine,
    fake_i10,
):
    RE(centre_tth(), docs)
    fake_step_scan_and_move_fit.assert_called_once_with(
        det=RASOR_DEFAULT_DET,
        motor=diffractometer().tth,
        start=-1,
        end=1,
        num=21,
        detname_suffix=RASOR_DEFAULT_DET_NAME_EXTENSION,
        fitted_loc=StatPosition.CEN,
    )


@patch("i10_bluesky.plans.centre_direct_beam.step_scan_and_move_fit")
async def test_centre_alpha(fake_step_scan_and_move_fit: Mock, RE: RunEngine, fake_i10):
    RE(centre_alpha())

    fake_step_scan_and_move_fit.assert_called_once_with(
        det=RASOR_DEFAULT_DET,
        motor=diffractometer().alpha,
        start=-0.8,
        end=0.8,
        num=21,
        detname_suffix=RASOR_DEFAULT_DET_NAME_EXTENSION,
        fitted_loc=StatPosition.CEN,
    )


@patch("i10_bluesky.plans.centre_direct_beam.step_scan_and_move_fit")
async def test_centre_det_angles(
    fake_step_scan_and_move_fit: Mock,
    RE: RunEngine,
):
    RE(centre_det_angles())
    assert fake_step_scan_and_move_fit.call_args_list[0] == call(
        det=RASOR_DEFAULT_DET,
        motor=diffractometer().tth,
        start=-1,
        end=1,
        num=21,
        detname_suffix=RASOR_DEFAULT_DET_NAME_EXTENSION,
        fitted_loc=StatPosition.CEN,
    )
    assert fake_step_scan_and_move_fit.call_args_list[1] == call(
        det=RASOR_DEFAULT_DET,
        motor=diffractometer().alpha,
        start=-0.8,
        end=0.8,
        num=21,
        detname_suffix=RASOR_DEFAULT_DET_NAME_EXTENSION,
        fitted_loc=StatPosition.CEN,
    )


def test_move_pin_origin_default():
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(move_pin_origin())
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().x, value=0)
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().y, value=0)
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().z, value=0)
    msgs = check_msg_wait(msgs=msgs, wait_group="move_pin_origin")
    assert len(msgs) == 1


def test_move_pin_origin_default_without_wait():
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(move_pin_origin(wait=False))
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().x, value=0)
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().y, value=0)
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().z, value=0)
    assert len(msgs) == 1
