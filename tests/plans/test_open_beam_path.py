from collections import defaultdict
from unittest.mock import ANY, Mock, patch

import pytest
from bluesky.run_engine import RunEngine
from bluesky.simulators import RunEngineSimulator
from dodal.beamlines.i10 import det_slits, pa_stage, pin_hole, slits
from ophyd_async.testing import (
    callback_on_mock_put,
    get_mock_put,
    set_mock_value,
)

from i10_bluesky.plans import (
    clear_beam_path,
    direct_beam_polan,
    open_dsd_dsu,
    open_s5s6,
    remove_pin_hole,
)
from i10_bluesky.plans.configuration.default_setting import (
    DSD_DSU_OPENING_POS,
    PIN_HOLE_OPEING_POS,
    S5S6_OPENING_SIZE,
)

from ..helper_functions import check_msg_set, check_msg_wait

docs = defaultdict(list)


def capture_emitted(name, doc):
    docs[name].append(doc)


async def test_open_s5s6_with_default(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(open_s5s6())
    msgs = check_msg_set(msgs=msgs, obj=slits().s5.x_gap, value=S5S6_OPENING_SIZE)
    msgs = check_msg_set(msgs=msgs, obj=slits().s5.y_gap, value=S5S6_OPENING_SIZE)
    msgs = check_msg_set(msgs=msgs, obj=slits().s6.x_gap, value=S5S6_OPENING_SIZE)
    msgs = check_msg_set(msgs=msgs, obj=slits().s6.y_gap, value=S5S6_OPENING_SIZE)
    group = f"{slits().name}__wait"
    msgs = check_msg_wait(msgs=msgs, wait_group=group)
    assert len(msgs) == 1


async def test_open_s5s6_with_other_size(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    other_value = 0.5
    msgs = sim.simulate_plan(open_s5s6(other_value))
    msgs = check_msg_set(msgs=msgs, obj=slits().s5.x_gap, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=slits().s5.y_gap, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=slits().s6.x_gap, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=slits().s6.y_gap, value=other_value)
    group = f"{slits().name}__wait"
    msgs = check_msg_wait(msgs=msgs, wait_group=group)
    assert len(msgs) == 1


async def test_open_s5s6_with_no_wait(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    other_value = 0.5
    msgs = sim.simulate_plan(open_s5s6(other_value, wait=False))
    msgs = check_msg_set(msgs=msgs, obj=slits().s5.x_gap, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=slits().s5.y_gap, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=slits().s6.x_gap, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=slits().s6.y_gap, value=other_value)
    assert len(msgs) == 1


async def test_open_dsd_dsu_with_default(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    msgs = sim.simulate_plan(open_dsd_dsu())
    msgs = check_msg_set(msgs=msgs, obj=det_slits().upstream, value=DSD_DSU_OPENING_POS)
    msgs = check_msg_set(
        msgs=msgs, obj=det_slits().downstream, value=DSD_DSU_OPENING_POS
    )
    group = f"{det_slits().name}_wait"
    msgs = check_msg_wait(msgs=msgs, wait_group=group)
    assert len(msgs) == 1


async def test_open_dsd_dsu_with_other_size(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    other_value = 0.5
    msgs = sim.simulate_plan(open_dsd_dsu(other_value))
    msgs = check_msg_set(msgs=msgs, obj=det_slits().upstream, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=det_slits().downstream, value=other_value)
    group = f"{det_slits().name}_wait"
    msgs = check_msg_wait(msgs=msgs, wait_group=group)
    assert len(msgs) == 1


async def test_open_dsd_dsu_with_no_wait(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    other_value = 0.5
    msgs = sim.simulate_plan(open_dsd_dsu(other_value, wait=False))
    msgs = check_msg_set(msgs=msgs, obj=det_slits().upstream, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=det_slits().downstream, value=other_value)
    assert len(msgs) == 1


@pytest.fixture
async def pinhole(fake_i10):
    ph = pin_hole()
    set_mock_value(ph.x.velocity, 2.78)
    set_mock_value(ph.x.user_readback, 1)

    return ph


async def test_remove_pin_hole_with_default(RE: RunEngine, pinhole):
    callback_on_mock_put(
        pinhole.x.user_setpoint,
        lambda *_, **__: set_mock_value(pinhole.x.user_readback, PIN_HOLE_OPEING_POS),
    )

    RE(remove_pin_hole())
    get_mock_put(pin_hole().x.user_setpoint).assert_called_once_with(
        PIN_HOLE_OPEING_POS, wait=ANY
    )
    assert await pin_hole().x.user_readback.get_value() == PIN_HOLE_OPEING_POS


async def test_remove_pin_hole_with_other_value(RE: RunEngine, pinhole):
    sim = RunEngineSimulator()
    other_value = 0.5
    msgs = sim.simulate_plan(remove_pin_hole(other_value, wait=True))
    msgs = check_msg_set(msgs=msgs, obj=pinhole.x, value=other_value)
    group = f"{pinhole.name}_wait"
    msgs = check_msg_wait(msgs=msgs, wait_group=group)
    assert len(msgs) == 1


async def test_remove_pin_hole_with_no_wait(RE: RunEngine, pinhole):
    sim = RunEngineSimulator()
    other_value = 0.5
    msgs = sim.simulate_plan(remove_pin_hole(other_value, wait=False))
    msgs = check_msg_set(msgs=msgs, obj=pinhole.x, value=other_value)
    assert len(msgs) == 1


async def test_direct_beam_polan(RE: RunEngine, fake_i10):
    sim = RunEngineSimulator()
    other_value = 0.0
    msgs = sim.simulate_plan(direct_beam_polan())
    msgs = check_msg_set(msgs=msgs, obj=pa_stage().eta, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=pa_stage().py, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=pa_stage().ttp, value=other_value)
    msgs = check_msg_set(msgs=msgs, obj=pa_stage().thp, value=other_value)
    group = f"{pa_stage().name}_wait"
    msgs = check_msg_wait(msgs=msgs, wait_group=group)
    assert len(msgs) == 1


@patch("i10_bluesky.plans.open_beam_path.direct_beam_polan")
@patch("i10_bluesky.plans.open_beam_path.open_dsd_dsu")
@patch("i10_bluesky.plans.open_beam_path.open_s5s6")
@patch("i10_bluesky.plans.open_beam_path.remove_pin_hole")
async def test_clear_beam_path(
    direct_beam_polan: Mock,
    open_dsd_dsu: Mock,
    open_s5s6: Mock,
    remove_pin_hole: Mock,
    RE: RunEngine,
):
    RE(clear_beam_path())
    direct_beam_polan.assert_called_once()
    open_dsd_dsu.assert_called_once()
    open_s5s6.assert_called_once()
    remove_pin_hole.assert_called_once()
