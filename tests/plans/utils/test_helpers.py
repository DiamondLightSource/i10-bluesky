import pytest

from i10_bluesky.plans.utils import cal_range_num


@pytest.mark.parametrize(
    "test_input, expected_output",
    [
        (
            [5, -5, 0.1],
            (10, 0, 200),
        ),
        (
            [50.6, 55.4, 15],
            (-4.8, 106, 15),
        ),
        (
            [
                -1.1,
                -2.5,
                31,
            ],
            (1.4, -3.6, 1),
        ),
    ],
)
def test_slit_cal_range_num(test_input, expected_output):
    assert cal_range_num(*test_input) == pytest.approx(expected_output, 0.01)
