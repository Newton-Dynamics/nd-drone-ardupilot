import time
import math


def set_throttle(pwm):
    """ Sends RC override to the throttle channel. """
    Script.SendRC(6, pwm, True)
    print(f"Throttle set to PWM: {pwm}")


def ramp_up(min_pwm, max_pwm, duration, steps):
    """ Linearly ramp up throttle from min_pwm to max_pwm """
    step_time = duration / steps
    for i in range(steps + 1):
        pwm_value = int(min_pwm + i * (max_pwm - min_pwm) / steps)
        set_throttle(pwm_value)
        time.sleep(step_time)


def ramp_down(max_pwm, min_pwm, duration, steps):
    """ Linearly ramp down throttle from max_pwm to min_pwm """
    step_time = duration / steps
    for i in range(steps + 1):
        pwm_value = int(max_pwm - i * (max_pwm - min_pwm) / steps)
        set_throttle(pwm_value)
        time.sleep(step_time)


def main():

    print("Starting ramp test...")

    # Throttle parameters
    min_pwm = 1000  # 0% throttle
    max_pwm = 2000  # 100% throttle
    duration_up = 1  # seconds to reach max throttle
    duration_down = 1  # seconds to ramp down throttle

    pwm_prec = 100  # Needed throttle in %
    hold_pwm = math.ceil(pwm_prec*(max_pwm-min_pwm)/100+min_pwm)

    steps = 100  # number of throttle steps for smooth ramp

    # Ramp up from 0% to 100%
    print("Ramp-up in progress...")
    ramp_up(min_pwm, hold_pwm, duration_up, steps)

    # Hold at 100% throttle for a short duration (optional)
    print("Throttle at 100% for a few seconds...")
    time.sleep(60)

    # Ramp down from 100% to 0%
    print("Ramp-down in progress...")
    ramp_down(hold_pwm, min_pwm, duration_down, steps)

    # Reset throttle to 0%
    set_throttle(min_pwm)
    print("Throttle test complete. Disarming vehicle...")


# Run the main function
main()
