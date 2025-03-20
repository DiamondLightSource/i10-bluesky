import pytest
from bluesky.run_engine import RunEngine
from p99_bluesky.devices.stages import ThreeAxisStage

from i10_bluesky.plans.utils import move_motor_with_look_up

fake_motor_look_up = {"5000": 1.8, "1000": 8, "-500": 8.8, "100": 55, "50": -34.3}


def test_motor_with_look_up_fail(RE: RunEngine, sim_motor_step: ThreeAxisStage):
    size = 400
    with pytest.raises(ValueError) as e:
        RE(
            move_motor_with_look_up(
                sim_motor_step.z, size=size, motor_table=fake_motor_look_up
            )
        )
    assert (
        str(e.value)
        == f"No slit with size={size}. Available slit size: {fake_motor_look_up}"
    )


def test_motor_with_look_up_fail_invalid_table(
    RE: RunEngine, sim_motor_step: ThreeAxisStage
):
    bad_motor_look_up = {"5000": 1.8, "1000": 8, "-500": 8.8, "100": "sdsf", "50": 34.3}

    size = 400
    with pytest.raises(ValueError):
        RE(
            move_motor_with_look_up(
                sim_motor_step.z, size=size, motor_table=bad_motor_look_up
            )
        )


@pytest.mark.parametrize(
    "test_input, expected_centre",
    [(5000, 1.8), (-500, 8.8), (50, -34.3)],
)
async def test_motor_with_look_up_move_using_table_success(
    RE: RunEngine, sim_motor_step: ThreeAxisStage, test_input, expected_centre
):
    RE(
        move_motor_with_look_up(
            sim_motor_step.z, size=test_input, motor_table=fake_motor_look_up
        )
    )
    assert await sim_motor_step.z.user_readback.get_value() == expected_centre


@pytest.mark.parametrize(
    "test_input, expected_centre",
    [(50, 50), (-5, -5), (0, 0)],
)
async def test_motor_with_look_up_move_using_motor_position_success(
    RE: RunEngine, sim_motor_step: ThreeAxisStage, test_input, expected_centre
):
    RE(
        move_motor_with_look_up(
            sim_motor_step.z,
            size=test_input,
            motor_table=fake_motor_look_up,
            use_motor_position=True,
        )
    )
    assert await sim_motor_step.z.user_readback.get_value() == expected_centre
