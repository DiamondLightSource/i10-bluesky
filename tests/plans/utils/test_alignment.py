from collections import defaultdict
from unittest.mock import Mock

import numpy as np
import pytest
from bluesky.run_engine import RunEngine
from ophyd_async.testing import callback_on_mock_put, set_mock_value
from p99_bluesky.devices.stages import ThreeAxisStage

from i10_bluesky.plans.utils import (
    StatPosition,
    align_slit_with_look_up,
    fast_scan_and_move_fit,
    step_scan_and_move_fit,
)
from i10_bluesky.plans.utils.helpers import cal_range_num
from sim_devices import sim_detector

docs = defaultdict(list)


def capture_emitted(name, doc):
    docs[name].append(doc)


def gaussian(x, mu, sig):
    return (
        1.0
        / (np.sqrt(2.0 * np.pi) * sig)
        * np.exp(-np.power((x - mu) / sig, 2.0) / 2.0)
    )


@pytest.mark.parametrize(
    "test_input, expected_centre",
    [
        (
            [5, -5, 21, 0.1],
            -1,
        ),
        (
            [50, -55, 151, 1],
            21.1,
        ),
        (
            [-1, -2.51241, 31, 0.05],
            -1.2,
        ),
    ],
)
async def test_scan_and_move_cen_success_with_gaussian(
    RE: RunEngine,
    sim_motor_step: ThreeAxisStage,
    fake_detector: sim_detector,
    test_input,
    expected_centre,
):
    start = test_input[0]
    end = test_input[1]
    num = test_input[2]
    peak_width = test_input[3]
    cen = expected_centre
    # Generate gaussian
    x_data = np.linspace(start, end, num, endpoint=True)
    y_data = gaussian(x_data, cen, peak_width)

    rbv_mocks = Mock()
    y_data = np.append(y_data, [0] * 2)
    y_data = np.array(y_data, dtype=np.float64)
    rbv_mocks.get.side_effect = y_data
    callback_on_mock_put(
        sim_motor_step.x.user_setpoint,
        lambda *_, **__: set_mock_value(fake_detector.value, value=rbv_mocks.get()),
    )
    docs = defaultdict(list)
    RE(
        step_scan_and_move_fit(
            fake_detector,
            sim_motor_step.x,
            StatPosition.COM,
            start,
            end,
            num,
        ),
        capture_emitted,
    )
    y_data1 = np.array([])
    x_data1 = np.array([])
    for i in docs["event"]:
        y_data1 = np.append(y_data1, i["data"]["fake_detector-value"])
        x_data1 = np.append(x_data1, i["data"]["sim_motor_step-x-user_readback"])
    assert await sim_motor_step.x.user_setpoint.get_value() == pytest.approx(
        expected_centre, 0.01
    )


def step_function(x_data, step_centre):
    return [0 if x < step_centre else 1 for x in x_data]


@pytest.mark.parametrize(
    "test_input, expected_centre",
    [
        (
            [3, -3, 101],
            -1,
        ),
        (
            [0.1, 0.5, 101],
            0.2,
        ),
    ],
)
async def test_scan_and_move_cen_success_with_step(
    RE: RunEngine,
    sim_motor_step: ThreeAxisStage,
    fake_detector: sim_detector,
    test_input,
    expected_centre,
):
    start = test_input[0]
    end = test_input[1]
    num = test_input[2]
    cen = expected_centre
    # Generate a step
    x_data = np.linspace(start, end, num, endpoint=True)
    y_data = step_function(x_data, cen)

    rbv_mocks = Mock()
    y_data = np.append(y_data, [0] * 2)
    y_data = np.array(y_data, dtype=np.float64)
    rbv_mocks.get.side_effect = y_data
    callback_on_mock_put(
        sim_motor_step.x.user_setpoint,
        lambda *_, **__: set_mock_value(fake_detector.value, value=rbv_mocks.get()),
    )
    docs = defaultdict(list)
    RE(
        step_scan_and_move_fit(
            det=fake_detector,
            motor=sim_motor_step.x,
            start=start,
            end=end,
            num=num,
            fitted_loc=StatPosition.D_CEN,
        ),
        capture_emitted,
    )
    y_data1 = np.array([])
    x_data1 = np.array([])
    for i in docs["event"]:
        y_data1 = np.append(y_data1, i["data"]["fake_detector-value"])
        x_data1 = np.append(x_data1, i["data"]["sim_motor_step-x-user_readback"])
    assert await sim_motor_step.x.user_setpoint.get_value() == pytest.approx(
        expected_centre, 0.05
    )


async def test_scan_and_move_cen_fail_to_with_wrong_name(
    RE: RunEngine,
    sim_motor_step: ThreeAxisStage,
    fake_detector: sim_detector,
):
    rbv_mocks = Mock()
    y_data = range(0, 19999, 1)
    rbv_mocks.get.side_effect = y_data
    callback_on_mock_put(
        sim_motor_step.x.user_setpoint,
        lambda *_, **__: set_mock_value(fake_detector.value, value=rbv_mocks.get()),
    )
    sim_motor_step.x._name = " "
    with pytest.raises(ValueError) as e:
        RE(
            fast_scan_and_move_fit(
                det=fake_detector,
                motor=sim_motor_step.x,
                start=-5,
                end=5,
                fitted_loc=StatPosition.CEN,
                motor_speed=100,
            ),
            capture_emitted,
        )

    assert str(e.value) == "Fitting failed, check devices name are correct."


@pytest.mark.parametrize(
    "test_input, expected_centre",
    [
        (
            [5, -4, 31, 0.1],
            -5.1,
        ),
    ],
)
async def test_scan_and_move_cen_failed_with_no_peak_in_range(
    RE: RunEngine,
    sim_motor_step: ThreeAxisStage,
    fake_detector: sim_detector,
    test_input,
    expected_centre,
):
    start = test_input[0]
    end = test_input[1]
    num = test_input[2]
    peak_width = test_input[3]
    cen = expected_centre
    # Generate gaussian
    x_data = np.linspace(start, end, num, endpoint=True)
    y_data = gaussian(x_data, cen, peak_width)

    rbv_mocks = Mock()
    y_data = np.append(y_data, [0] * 2)
    y_data = np.array(y_data, dtype=np.float64)
    rbv_mocks.get.side_effect = y_data
    callback_on_mock_put(
        sim_motor_step.x.user_setpoint,
        lambda *_, **__: set_mock_value(fake_detector.value, value=rbv_mocks.get()),
    )
    with pytest.raises(ValueError) as e:
        RE(
            step_scan_and_move_fit(
                det=fake_detector,
                motor=sim_motor_step.x,
                start=start,
                end=end,
                fitted_loc=StatPosition.CEN,
                num=num,
            ),
        )
    assert str(e.value) == "Fitting failed, no peak within scan range."


FAKEDSU = {"5000": 16.7, "1000": 21.7, "500": 25.674, "100": 31.7, "50": 36.7}


@pytest.mark.parametrize(
    "size, expected_centre, offset",
    [
        (5000, 16.7, 0.1),
        (1000, 21.7, 0.2),
        (500, 25.6, 0.2),
    ],
)
async def test_align_slit_with_look_up(
    RE: RunEngine,
    sim_motor_step: ThreeAxisStage,
    fake_detector: sim_detector,
    size,
    expected_centre,
    offset,
):
    start, end, num = cal_range_num(
        cen=FAKEDSU[str(size)], range=size / 1000 * 3, size=size / 5000.0
    )
    peak_width = FAKEDSU[str(size)] * 0.5
    cen = FAKEDSU[str(size)] + offset
    # Generate gaussian
    x_data = np.linspace(start, end, num, endpoint=True)
    y_data = gaussian(x_data, cen, peak_width)

    rbv_mocks = Mock()
    y_data = np.append(y_data, [0] * 2)
    y_data = np.array(y_data, dtype=np.float64)
    rbv_mocks.get.side_effect = y_data
    callback_on_mock_put(
        sim_motor_step.y.user_setpoint,
        lambda *_, **__: set_mock_value(fake_detector.value, value=rbv_mocks.get()),
    )
    docs = defaultdict(list)
    RE(
        align_slit_with_look_up(
            motor=sim_motor_step.y,
            size=size,
            slit_table=FAKEDSU,
            det=fake_detector,
            centre_type=StatPosition.COM,
        ),
        capture_emitted,
    )
    y_data1 = np.array([])
    x_data1 = np.array([])
    for i in docs["event"]:
        y_data1 = np.append(y_data1, i["data"]["fake_detector-value"])
        x_data1 = np.append(x_data1, i["data"]["sim_motor_step-y-user_readback"])
    assert FAKEDSU[str(size)] == pytest.approx(expected_centre + offset, 0.01)


async def test_align_slit_with_look_up_fail_wrong_key(
    RE: RunEngine,
    sim_motor_step: ThreeAxisStage,
    fake_detector: sim_detector,
):
    size = 555
    with pytest.raises(ValueError) as e:
        RE(
            align_slit_with_look_up(
                motor=sim_motor_step.y,
                size=size,
                slit_table=FAKEDSU,
                det=fake_detector,
                centre_type=StatPosition.CEN,
            ),
        )
    assert str(e.value) == f"Size of {size} is not in {FAKEDSU.keys}"
