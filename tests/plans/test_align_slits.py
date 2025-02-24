from collections import defaultdict
from unittest.mock import ANY, MagicMock, patch

import pytest
from bluesky.simulators import RunEngineSimulator, assert_message_and_return_remaining
from dodal.beamlines.i10 import (
    det_slits,
    diffractometer,
    rasor_femto_pa_scaler_det,
    simple_stage,
    slits,
)

from i10_bluesky.plans.align_slits import (
    DSD,
    DSU,
    align_pa_slit,
    align_s5s6,
    align_slit,
    move_dsd,
    move_dsu,
)

docs = defaultdict(list)


def capture_emitted(name, doc):
    docs[name].append(doc)


def test_move_dsu():
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(move_dsu(5000))
    msgs = check_msg_set(msgs=msgs, obj=det_slits().upstream, value=DSU["5000"])
    msgs = check_msg_wait(msgs=msgs, wait_group=ANY, wait=True)
    assert len(msgs) == 1


def test_move_dsd():
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(move_dsd(50))
    msgs = check_msg_set(msgs=msgs, obj=det_slits().downstream, value=DSD["50"])
    msgs = check_msg_wait(msgs=msgs, wait_group=ANY, wait=True)
    assert len(msgs) == 1


@patch(
    "i10_bluesky.plans.align_slits.align_slit_with_look_up",
)
def test_align_pa_slit(fake_step_scan_and_move_fit: MagicMock):
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(align_pa_slit(dsd_size=50, dsu_size=50))
    msgs = check_msg_set(msgs=msgs, obj=det_slits().downstream, value=DSD["5000"])
    msgs = check_msg_wait(msgs=msgs, wait_group=ANY, wait=True)

    assert len(msgs) == 1
    assert fake_step_scan_and_move_fit.call_count == 2


@patch(
    "i10_bluesky.plans.align_slits.align_slit",
)
def test_align_s5s6(mock_align_slit: MagicMock):
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(align_s5s6())
    assert mock_align_slit.call_count == 2
    msgs = check_msg_set(msgs=msgs, obj=diffractometer().tth, value=0)
    msgs = check_msg_set(msgs=msgs, obj=diffractometer().th, value=0)
    msgs = check_msg_set(msgs=msgs, obj=simple_stage().y, value=-3)
    msgs = check_msg_wait(msgs=msgs, wait_group="diff group A")
    assert len(msgs) == 1


@patch("i10_bluesky.plans.align_slits.cal_range_num", return_value=[1, 1, 1])
@patch(
    "i10_bluesky.plans.align_slits.step_scan_and_move_fit",
)
@pytest.mark.parametrize(
    """x_scan_size, x_final_size, x_open_size, y_scan_size, y_final_size,
      y_open_size, x_range, x_cen, y_range, y_cen""",
    [
        (0, 1, 2, 3.4, 5, 6, 7, 8, 9, 10),
    ],
)
def test_align_slit(
    mock_step_scan: MagicMock,
    mock_cal_range: MagicMock,
    x_scan_size,
    x_final_size,
    x_open_size,
    y_scan_size,
    y_final_size,
    y_open_size,
    x_range,
    x_cen,
    y_range,
    y_cen,
):
    slit = slits().s5
    det = rasor_femto_pa_scaler_det()
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(
        align_slit(
            det,
            slit,
            x_scan_size,
            x_final_size,
            x_open_size,
            y_scan_size,
            y_final_size,
            y_open_size,
            x_range,
            x_cen,
            y_range,
            y_cen,
        )
    )
    msgs = check_msg_set(msgs=msgs, obj=slit.x_gap, value=x_scan_size)
    msgs = check_msg_set(msgs=msgs, obj=slit.y_gap, value=y_open_size)
    msgs = check_msg_wait(msgs=msgs, wait_group="slits group")
    msgs = check_msg_set(msgs=msgs, obj=slit.y_centre, value=y_cen)
    msgs = check_mv_wait(msgs=msgs, wait_group=ANY)
    msgs = check_msg_set(msgs=msgs, obj=slit.y_gap, value=y_scan_size)

    msgs = check_msg_set(msgs=msgs, obj=slit.x_gap, value=x_open_size)
    msgs = check_msg_wait(msgs=msgs, wait_group="slits group")
    msgs = check_msg_set(msgs=msgs, obj=slit.x_gap, value=x_final_size)
    msgs = check_msg_set(msgs=msgs, obj=slit.y_gap, value=y_final_size)
    msgs = check_msg_wait(msgs=msgs, wait_group="slits group")
    assert len(msgs) == 1
    assert mock_step_scan.call_count == 2
    assert mock_cal_range.call_count == 2


def check_msg_set(msgs, obj, value):
    return assert_message_and_return_remaining(
        msgs,
        lambda msg: msg.command == "set" and msg.obj is obj and msg.args[0] == value,
    )


def check_msg_wait(msgs, wait_group, wait=False):
    wait_msg = (
        {"group": wait_group}
        if wait
        else {"group": wait_group, "move_on": False, "timeout": None}
    )
    return assert_message_and_return_remaining(
        msgs,
        lambda msg: msg.command == "wait"
        and msg.obj is None
        and msg.kwargs == wait_msg,
    )


def check_mv_wait(msgs, wait_group):
    return assert_message_and_return_remaining(
        msgs,
        lambda msg: msg.command == "wait"
        and msg.obj is None
        and msg.kwargs == {"group": wait_group},
    )
