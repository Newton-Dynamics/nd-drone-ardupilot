-- Author: Hussein Sleiman

-- Drone Version: NewtonTwo - Series 1 (ND21)
-- Published in 06 August 2026
-- Updated in 25 August 2026

-- ================================ Description ============================================
-- Forward motor stow control for ArduPlane 4.6.3 QuadPlane
--
-- Purpose:
--   Smoothly ramp the horizontal motor (SERVO13 / function 70) down to
--   SERVO13_MIN when the low-altitude / low-horizontal-speed stow conditions
--   are met, and smoothly ramp it back toward ArduPilot's requested throttle
--   before returning full control to ArduPilot.
--
-- Messages are intentionally limited to:
--   1) INITIALIZED
--   2) TRIGGERED
--   3) RELEASED
--
-- State machine:
--   NORMAL -> RAMP_DOWN -> STOWED -> RAMP_UP -> NORMAL
--
-- Additional behaviour:
-- If a stow condition becomes invalid during RAMP_DOWN, immediately reverse direction and enter RAMP_UP
-- If the stow conditions become valid again during RAMP_UP and remain valid for the debounce period, reverse back into RAMP_DOWN.
-- ============================================================================

local SCRIPT_NAME = "FWD motor stow"

-- ========================= Configuration =========================
local THROTTLE_FUNCTION          = 70       -- SERVOx_FUNCTION = 70 (Throttle)
local EXPECTED_SERVO_OUTPUT      = 3       -- horizontal motor must be SERVO13

local ALT_LIMIT_M                = 4.0
local MAX_ALLOWED_N_E_VEL        = 0.2

local LOOP_MS                    = 50       -- 20 Hz
local OVERRIDE_TIMEOUT_MS        = 250      -- refreshed every loop while script owns output
local ENGAGE_DEBOUNCE_SAMPLES    = 10       -- 0.5 s at 20 Hz

-- Ramp rates. These only apply while entering/leaving the stow condition.
-- They do NOT limit normal ArduPilot throttle during the rest of the flight.
local RAMP_DOWN_PWM_PER_SEC      = 200.0    -- e.g. 1500 -> 1000 in 2.0 s
local RAMP_UP_PWM_PER_SEC        = 250.0    -- e.g. 1000 -> 1500 in ~1.43 s

-- Once the script-controlled PWM is this close to ArduPilot's requested PWM,
-- the override is cleared and ArduPilot resumes direct control.
local HANDOVER_TOLERANCE_PWM     = 5.0
-- ================================================================

-- ArduPlane 4.6.3 mode numbers
local MODE_AUTO     = 10
local MODE_QLOITER  = 19
local MODE_QLAND    = 20
local MODE_QRTL     = 21

-- MAV_CMD_NAV_VTOL_TAKEOFF
local MAV_CMD_NAV_VTOL_TAKEOFF = 84

-- State machine
local STATE_NORMAL    = 0
local STATE_RAMP_DOWN = 1
local STATE_STOWED    = 2
local STATE_RAMP_UP   = 3

local state = STATE_NORMAL
local trigger_active = false
local valid_sample_count = 0
local override_pwm = nil

-- Resolve the output channel from the assigned servo FUNCTION.
-- ArduPilot returns a zero-based internal channel index here.
local throttle_channel = assert(
    SRV_Channels:find_channel(THROTTLE_FUNCTION),
    "No servo output is assigned SERVOx_FUNCTION=70"
)

-- Verify that function 70 is actually on SERVO13.
assert(
    throttle_channel == (EXPECTED_SERVO_OUTPUT - 1),
    string.format(
        "Throttle function 70 resolved to SERVO%d, expected SERVO%d",
        throttle_channel + 1,
        EXPECTED_SERVO_OUTPUT
    )
)

-- Parameters
local q_fwd_thr_use = Parameter()
assert(
    q_fwd_thr_use:init("Q_FWD_THR_USE"),
    "Q_FWD_THR_USE parameter unavailable"
)

local servo_min = Parameter()
assert(
    servo_min:init("SERVO13_MIN"),
    "SERVO13_MIN parameter unavailable"
)

local servo_max = Parameter()
assert(
    servo_max:init("SERVO13_MAX"),
    "SERVO13_MAX parameter unavailable"
)

local servo_reversed = Parameter()
assert(
    servo_reversed:init("SERVO13_REVERSED"),
    "SERVO13_REVERSED parameter unavailable"
)

local function rounded_parameter_value(parameter_object)
    local value = parameter_object:get()
    if value == nil then
        return nil
    end
    return math.floor(value + 0.5)
end

local function clamp(value, minimum, maximum)
    if value < minimum then
        return minimum
    end
    if value > maximum then
        return maximum
    end
    return value
end

local function abs(value)
    if value < 0 then
        return -value
    end
    return value
end

-- For this application zero throttle is explicitly SERVO13_MIN (1000 PWM),
-- therefore SERVO13 must not be reversed.
local reversed_value = rounded_parameter_value(servo_reversed)
assert(
    reversed_value == 0,
    "SERVO13_REVERSED must be 0 for this stow script"
)

-- Validate servo endpoints once at initialization.
local init_min_pwm = rounded_parameter_value(servo_min)
local init_max_pwm = rounded_parameter_value(servo_max)
assert(init_min_pwm ~= nil and init_max_pwm ~= nil,
       "Unable to read SERVO13_MIN/MAX")
assert(init_max_pwm > init_min_pwm,
       "SERVO13_MAX must be greater than SERVO13_MIN")

-- Height above HOME in metres.
-- NED Down is positive, therefore altitude above home = -Down.
-- This is used for SITL because HAGL availability depends on configuration.
local function get_altitude_above_home_m()
    local pos_ned_home = ahrs:get_relative_position_NED_home()
    if pos_ned_home == nil then
        return nil
    end

    return -pos_ned_home:z()
end

local function allowed_flight_condition(mode)
    if mode == MODE_QLOITER or mode == MODE_QLAND then
        return true
    end

    if mode == MODE_AUTO then
        -- AUTO is allowed during:
        --   1) NAV_VTOL_TAKEOFF
        --   2) VTOL landing descent
        local current_nav_id = mission:get_current_nav_id()

        local auto_vtol_takeoff =
            current_nav_id == MAV_CMD_NAV_VTOL_TAKEOFF

        local auto_vtol_landing =
            quadplane:in_vtol_land_descent()

        return auto_vtol_takeoff or auto_vtol_landing
    end

    if mode == MODE_QRTL then
        -- QRTL is allowed during:
        --   1) VTOL landing descent
        return quadplane:in_vtol_land_descent()
    end

    return false
end

-- Exact stow condition from the user's original script.
-- BOTH VN and VE must remain inside the limit. If either exceeds the limit,
-- the function returns false and the script leaves the stow condition.
local function conditions_are_valid()
    if not arming:is_armed() then
        return false
    end

    local fwd_thr_use = q_fwd_thr_use:get()
    if fwd_thr_use == nil or fwd_thr_use <= 0 then
        return false
    end

    local mode = vehicle:get_mode()
    if mode == nil or not allowed_flight_condition(mode) then
        return false
    end

    local altitude_m = get_altitude_above_home_m()
    if altitude_m == nil or altitude_m >= ALT_LIMIT_M then
        return false
    end

    local velocity_ned = ahrs:get_velocity_NED()
    if velocity_ned == nil then
        return false
    end

    local vn = velocity_ned:x()
    local ve = velocity_ned:y()

    if vn == nil or ve == nil then
        return false
    end

    if math.abs(vn) > MAX_ALLOWED_N_E_VEL or
       math.abs(ve) > MAX_ALLOWED_N_E_VEL then
        return false
    end

    return true
end

-- Return ArduPilot's underlying throttle demand as PWM even while the raw
-- servo channel is being overridden by this script.
--
-- For ArduPlane function 70, get_output_scaled() is throttle in percent
-- (0..100). Map that demand to SERVO13_MIN..SERVO13_MAX.
local function get_ardupilot_requested_pwm()
    local minimum_pwm = rounded_parameter_value(servo_min)
    local maximum_pwm = rounded_parameter_value(servo_max)

    if minimum_pwm == nil or maximum_pwm == nil or maximum_pwm <= minimum_pwm then
        return nil
    end

    local throttle_pct = SRV_Channels:get_output_scaled(THROTTLE_FUNCTION)
    if throttle_pct == nil then
        return nil
    end

    throttle_pct = clamp(throttle_pct, 0.0, 100.0)

    return minimum_pwm +
           (maximum_pwm - minimum_pwm) * (throttle_pct / 100.0)
end

local function get_current_servo_pwm()
    local pwm = SRV_Channels:get_output_pwm_chan(throttle_channel)
    if pwm ~= nil then
        return pwm
    end

    return get_ardupilot_requested_pwm()
end

local function apply_override(pwm)
    local minimum_pwm = rounded_parameter_value(servo_min) or init_min_pwm
    local maximum_pwm = rounded_parameter_value(servo_max) or init_max_pwm

    pwm = clamp(pwm, minimum_pwm, maximum_pwm)
    local pwm_integer = math.floor(pwm + 0.5)

    SRV_Channels:set_output_pwm_chan_timeout(
        throttle_channel,
        pwm_integer,
        OVERRIDE_TIMEOUT_MS
    )

    override_pwm = pwm
end

local function clear_override_at_pwm(pwm)
    local minimum_pwm = rounded_parameter_value(servo_min) or init_min_pwm
    local maximum_pwm = rounded_parameter_value(servo_max) or init_max_pwm

    pwm = clamp(pwm, minimum_pwm, maximum_pwm)
    local pwm_integer = math.floor(pwm + 0.5)

    -- timeout=0 clears the channel override. Supplying a PWM already matched
    -- to ArduPilot's demand avoids a discontinuity at handover.
    SRV_Channels:set_output_pwm_chan_timeout(
        throttle_channel,
        pwm_integer,
        0
    )

    override_pwm = nil
end

local function move_toward(current_value, target_value, max_step)
    if current_value < target_value then
        return math.min(current_value + max_step, target_value)
    end

    if current_value > target_value then
        return math.max(current_value - max_step, target_value)
    end

    return target_value
end

local RAMP_DOWN_STEP_PWM = RAMP_DOWN_PWM_PER_SEC * (LOOP_MS / 1000.0)
local RAMP_UP_STEP_PWM   = RAMP_UP_PWM_PER_SEC   * (LOOP_MS / 1000.0)

local function send_triggered_message()
    local minimum_pwm = rounded_parameter_value(servo_min) or init_min_pwm
    gcs:send_text(
        5,
        string.format(
            "%s: TRIGGERED - ramping SERVO13 toward %d PWM", 
            SCRIPT_NAME,
            minimum_pwm
        )
    )
end

local function send_released_message()
    gcs:send_text(
        5,
        string.format(
            "%s: RELEASED - ramping SERVO13 back toward ArduPilot throttle",
            SCRIPT_NAME
        )
    )
end

local function begin_ramp_down()
    if override_pwm == nil then
        override_pwm = get_current_servo_pwm()
    end

    if override_pwm == nil then
        return false
    end

    -- Grab control at the current output first, so entering the override does
    -- not itself introduce a PWM step.
    apply_override(override_pwm)
    state = STATE_RAMP_DOWN

    if not trigger_active then
        trigger_active = true
        send_triggered_message()
    end

    return true
end

local function begin_ramp_up()
    state = STATE_RAMP_UP
    valid_sample_count = 0

    if trigger_active then
        trigger_active = false
        send_released_message()
    end
end

local function update_normal(valid)
    if not valid then
        valid_sample_count = 0
        return
    end

    if valid_sample_count < ENGAGE_DEBOUNCE_SAMPLES then
        valid_sample_count = valid_sample_count + 1
    end

    if valid_sample_count >= ENGAGE_DEBOUNCE_SAMPLES then
        valid_sample_count = 0
        begin_ramp_down()
    end
end

local function update_ramp_down(valid)
    -- Additional safety behaviour: if any stow condition becomes false while
    -- ramping down, immediately reverse into RAMP_UP instead of continuing
    -- toward zero throttle.
    if not valid then
        begin_ramp_up()
        return
    end

    local minimum_pwm = rounded_parameter_value(servo_min) or init_min_pwm

    if override_pwm == nil then
        override_pwm = get_current_servo_pwm() or minimum_pwm
    end

    override_pwm = move_toward(
        override_pwm,
        minimum_pwm,
        RAMP_DOWN_STEP_PWM
    )

    apply_override(override_pwm)

    if override_pwm <= minimum_pwm then
        override_pwm = minimum_pwm
        apply_override(override_pwm)
        state = STATE_STOWED
    end
end

local function update_stowed(valid)
    local minimum_pwm = rounded_parameter_value(servo_min) or init_min_pwm

    if not valid then
        begin_ramp_up()
        return
    end

    override_pwm = minimum_pwm
    apply_override(override_pwm)
end

local function update_ramp_up(valid)
    -- If stow conditions return during RAMP_UP, require the same debounce
    -- before reversing back into RAMP_DOWN. While waiting, keep the output
    -- smoothly matched toward ArduPilot's demand; do not hand over control
    -- until the re-trigger decision is resolved.
    if valid then
        if valid_sample_count < ENGAGE_DEBOUNCE_SAMPLES then
            valid_sample_count = valid_sample_count + 1
        end
    else
        valid_sample_count = 0
    end

    local target_pwm = get_ardupilot_requested_pwm()
    if target_pwm == nil then
        -- Conservative fallback: retain the current override and try again on
        -- the next loop rather than releasing to an unknown target.
        if override_pwm ~= nil then
            apply_override(override_pwm)
        end
        return
    end

    if override_pwm == nil then
        override_pwm = get_current_servo_pwm() or target_pwm
    end

    -- During handover, track a changing ArduPilot target smoothly in either
    -- direction. Upward changes use RAMP_UP rate; downward target changes use
    -- the RAMP_DOWN rate.
    if target_pwm >= override_pwm then
        override_pwm = move_toward(
            override_pwm,
            target_pwm,
            RAMP_UP_STEP_PWM
        )
    else
        override_pwm = move_toward(
            override_pwm,
            target_pwm,
            RAMP_DOWN_STEP_PWM
        )
    end

    apply_override(override_pwm)

    -- Re-enter stow if the trigger returned and persisted through debounce.
    if valid_sample_count >= ENGAGE_DEBOUNCE_SAMPLES then
        valid_sample_count = 0
        state = STATE_RAMP_DOWN

        if not trigger_active then
            trigger_active = true
            send_triggered_message()
        end
        return
    end

    -- If we are not waiting on a possible re-trigger and we have smoothly
    -- reached ArduPilot's requested PWM, clear the raw servo override.
    if not valid and
       abs(override_pwm - target_pwm) <= HANDOVER_TOLERANCE_PWM then
        clear_override_at_pwm(target_pwm)
        state = STATE_NORMAL
        valid_sample_count = 0
    end
end

local function update()
    local valid = conditions_are_valid()

    if state == STATE_NORMAL then
        update_normal(valid)

    elseif state == STATE_RAMP_DOWN then
        update_ramp_down(valid)

    elseif state == STATE_STOWED then
        update_stowed(valid)

    elseif state == STATE_RAMP_UP then
        update_ramp_up(valid)

    else
        -- Defensive recovery if state memory is corrupted.
        state = STATE_NORMAL
        trigger_active = false
        valid_sample_count = 0
        override_pwm = nil
    end

    return update, LOOP_MS
end

gcs:send_text(
    6,
    string.format(
        "%s: INITIALIZED - SERVO13 ramp stow control active",
        SCRIPT_NAME
    )
)

return update, 1000