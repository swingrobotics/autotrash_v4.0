def drive_pwm_magnitude(throttle, boost_active=False, minimum_start_pwm=80):
    magnitude = max(0.0, min(1.0, abs(float(throttle))))
    if magnitude == 0.0:
        return 0
    requested_pwm = min(255, int(magnitude * 255.0 + 0.5))
    if boost_active:
        return max(int(minimum_start_pwm), requested_pwm)
    return requested_pwm
