import asyncio
from pathlib import Path

import pytest
from bluesky.run_engine import RunEngine
from dodal.utils import make_all_devices
from ophyd_async.core import (
    FilenameProvider,
    StaticFilenameProvider,
    StaticPathProvider,
    init_devices,
)
from ophyd_async.testing import set_mock_value
from p99_bluesky.devices.stages import ThreeAxisStage
from p99_bluesky.sim.sim_stages import SimThreeAxisStage
from super_state_machine.errors import TransitionError

from .sim_devices import sim_detector

RECORD = str(Path(__file__).parent / "panda" / "db" / "panda.db")
INCOMPLETE_BLOCK_RECORD = str(
    Path(__file__).parent / "panda" / "db" / "incomplete_block_panda.db"
)
INCOMPLETE_RECORD = str(Path(__file__).parent / "panda" / "db" / "incomplete_panda.db")
EXTRA_BLOCKS_RECORD = str(
    Path(__file__).parent / "panda" / "db" / "extra_blocks_panda.db"
)


@pytest.fixture(scope="session")
def RE(request):
    loop = asyncio.new_event_loop()
    loop.set_debug(True)
    RE = RunEngine({}, call_returns_result=True, loop=loop)

    def clean_event_loop():
        if RE.state not in ("idle", "panicked"):
            try:
                RE.halt()
            except TransitionError:
                pass
        loop.call_soon_threadsafe(loop.stop)
        RE._th.join()
        loop.close()

    request.addfinalizer(clean_event_loop)
    return RE


A_BIT = 0.5


@pytest.fixture
def static_filename_provider():
    return StaticFilenameProvider("ophyd_async_tests")


@pytest.fixture
def static_path_provider_factory(tmp_path: Path):
    def create_static_dir_provider_given_fp(fp: FilenameProvider):
        return StaticPathProvider(fp, tmp_path)

    return create_static_dir_provider_given_fp


@pytest.fixture
def static_path_provider(
    static_path_provider_factory,
    static_filename_provider: FilenameProvider,
):
    return static_path_provider_factory(static_filename_provider)


@pytest.fixture
async def sim_motor():
    async with init_devices(mock=True):
        sim_motor = ThreeAxisStage("BLxxI-MO-TABLE-01:X", name="sim_motor")
    set_mock_value(sim_motor.x.velocity, 2.78)
    set_mock_value(sim_motor.x.high_limit_travel, 8.168)
    set_mock_value(sim_motor.x.low_limit_travel, -8.888)
    set_mock_value(sim_motor.x.user_readback, 1)
    set_mock_value(sim_motor.x.motor_egu, "mm")
    set_mock_value(sim_motor.x.motor_done_move, True)
    set_mock_value(sim_motor.x.max_velocity, 10)

    set_mock_value(sim_motor.y.motor_egu, "mm")
    set_mock_value(sim_motor.y.high_limit_travel, 5.168)
    set_mock_value(sim_motor.y.low_limit_travel, -5.888)
    set_mock_value(sim_motor.y.user_readback, 0)
    set_mock_value(sim_motor.y.motor_egu, "mm")
    set_mock_value(sim_motor.y.velocity, 2.88)
    set_mock_value(sim_motor.y.motor_done_move, True)
    set_mock_value(sim_motor.y.max_velocity, 10)

    set_mock_value(sim_motor.z.motor_egu, "mm")
    set_mock_value(sim_motor.z.high_limit_travel, 5.168)
    set_mock_value(sim_motor.z.low_limit_travel, -5.888)
    set_mock_value(sim_motor.z.user_readback, 0)
    set_mock_value(sim_motor.z.motor_egu, "mm")
    set_mock_value(sim_motor.z.velocity, 2.88)
    set_mock_value(sim_motor.z.motor_done_move, True)
    set_mock_value(sim_motor.z.max_velocity, 10)

    yield sim_motor


@pytest.fixture
async def sim_motor_step():
    async with init_devices(mock=True):
        sim_motor_step = SimThreeAxisStage(name="sim_motor_step", instant=True)

    yield sim_motor_step


@pytest.fixture
async def fake_detector():
    async with init_devices(mock=True):
        fake_detector = sim_detector(prefix="fake_Pv", name="fake_detector")
    set_mock_value(fake_detector.value, 0)
    yield fake_detector


@pytest.fixture
async def fake_i10():
    fake_i10, _ = make_all_devices(
        "dodal.beamlines.i10", connect_immediately=True, mock=True
    )
    yield fake_i10
