def signed_raw_delta(from_raw, to_raw):
    difference = int(to_raw) - int(from_raw)
    if difference > 2047:
        difference -= 4096
    elif difference < -2048:
        difference += 4096
    return difference


def calibration_valid(right_raw, center_raw, left_raw, allowance_raw):
    values = (right_raw, center_raw, left_raw)
    if any(value < 0 or value >= 4096 for value in values):
        return False
    if allowance_raw < 0 or allowance_raw > 300:
        return False
    right_delta = signed_raw_delta(center_raw, right_raw)
    left_delta = signed_raw_delta(center_raw, left_raw)
    return (
        right_delta != 0
        and left_delta != 0
        and right_delta * left_delta < 0
        and abs(right_delta) + allowance_raw < 2048
        and abs(left_delta) + allowance_raw < 2048
    )


def safety_limit(reference_raw, center_raw, allowance_raw):
    direction = 1 if signed_raw_delta(center_raw, reference_raw) >= 0 else -1
    return (int(reference_raw) + direction * int(allowance_raw)) % 4096

