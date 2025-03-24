import math


def cal_range_num(cen, range, size) -> tuple[float, float, int]:
    """Calculate the start, end and the number of step for scan."""
    start_pos = cen - range
    end_pos = cen + range
    num = math.ceil(abs(range * 4.0 / size))
    return start_pos, end_pos, num
