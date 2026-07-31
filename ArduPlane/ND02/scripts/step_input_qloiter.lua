-- Author: Hussein Sleiman
-- Release: February 25, 2026

-- ====================================Description ===================================================
-- This script is to automate a maneuver during flight by overriding the pilot stick input.
-- This maneuver have a "Step" ramp up/down shape in VTOL and can only be activated in QHover.

-- To activate the script:
-- CH5 switch UP   --> Activate script
-- CH5 switch DOWN --> Deactivate script

-- To select roll, pitch, or yaw:
-- CH6 switch --> UP  = ROLL
--                MID = PITCH
--                LOW = YAW

-- To select direction:
-- CH9  switch --> UP   = roll to right, pitch backward, yaw right
--             --> DOWN = roll to left, pitch forward, yaw left
-- ===================================================================================================


local SCRIPT_NAME = "step_input"

-- ======================== User configuration ========================

-- Maneuver direction:
--   1  = positive RC input
--  -1  = negative RC input
local INPUT_SIGN = 1

-- Trigger switch
local TRIG_CH      = 9
local TRIG_HIGH_US = 1850
local TRIG_LOW_US  = 1100

-- Axis-selection switch
local SEL_CH       = 11
local SEL_HIGH_MIN = 1700

-- RC input channels:
-- Roll  = RC input channel 1
-- Pitch = RC input channel 2
local ROLL_IN  = 1
local PITCH_IN = 2

-- Step definition
local T_STEP_MS = 5000
local AMP_US    = 1000

-- Set to 0 for a true step.
-- Set above 0 to ramp from the baseline to the target.
local FADE_IN_MS = 0

local UPDATE_MS = 20

-- ArduPlane QuadPlane flight-mode numbers
-- local MODE_QHOVER  = 18
local MODE_QLOITER = 19

-- Trigger debounce and rearm logic
local REARM_LOW_MS = 300
local ABORT_LOW_MS = 150

-- ===================================================================

local SEV_INFO = 6

local function gcs_msg(txt)
    gcs:send_text(
        SEV_INFO,
        string.format("%s: %s", SCRIPT_NAME, txt)
    )
end

local function clamp(x, lo, hi)
    if x < lo then
        return lo
    elseif x > hi then
        return hi
    end

    return x
end

local function pget(name, default)
    local value = param:get(name)
    return tonumber(value) or default
end

local function rcpwm(channel_number)
    local value = rc:get_pwm(channel_number)
    return tonumber(value) or 0
end

local function input_sign()
    if INPUT_SIGN >= 0 then
        return 1
    end

    return -1
end

local function in_allowed_mode(mode)
    return mode == MODE_QHOVER or mode == MODE_QLOITER
end

local function clear_overrides()
    -- Only roll and pitch can be overridden by this script.
    local channels = {
        ROLL_IN,
        PITCH_IN
    }

    for i = 1, #channels do
        local channel = rc:get_channel(channels[i])

        if channel then
            if channel.clear_override then
                channel:clear_override()
            else
                -- Fallback for older firmware versions.
                channel:set_override(0)
            end
        end
    end

    -- Additional global cleanup when supported.
    if rc and rc.clear_overrides then
        rc:clear_overrides()
    end
end

local function rc_limits(channel_number)
    local minimum = pget(
        string.format("RC%d_MIN", channel_number),
        1000
    )

    local trim = pget(
        string.format("RC%d_TRIM", channel_number),
        1500
    )

    local maximum = pget(
        string.format("RC%d_MAX", channel_number),
        2000
    )

    return minimum, trim, maximum
end

local function select_axis()
    local selector_pwm = rcpwm(SEL_CH)

    -- CH6 UP selects roll.
    if selector_pwm >= SEL_HIGH_MIN then
        return ROLL_IN, "ROLL"
    end

    -- CH6 MID or DOWN selects pitch.
    return PITCH_IN, "PITCH"
end

local function set_rc_override(channel_number, pwm)
    local channel = rc:get_channel(channel_number)

    if channel then
        channel:set_override(
            math.floor((tonumber(pwm) or 0) + 0.5)
        )
    end
end

-- ============================== State ==============================

local running = false
local armed   = false

local trigger_low_since_ms = nil
local abort_low_since_ms   = nil

local t0_ms        = 0
local axis_ch      = nil
local axis_lbl     = ""
local baseline_pwm = 1500
local target_pwm   = 1500

-- ===================================================================

function update()
    local now_ms = millis()
    local mode   = vehicle:get_mode() or -1
    local trig   = rcpwm(TRIG_CH)

    -- Match the switch description:
    -- CH5 UP   = activate
    -- CH5 DOWN = stop/rearm
    local trig_high = trig >= TRIG_HIGH_US
    local trig_low  = trig <= TRIG_LOW_US

    ------------------------------------------------------------------
    -- Trigger-low debounce for rearming
    ------------------------------------------------------------------

    if trig_low then
        if not trigger_low_since_ms then
            trigger_low_since_ms = now_ms
        end
    else
        trigger_low_since_ms = nil
    end

    if not running then
        if trigger_low_since_ms and
           ((now_ms - trigger_low_since_ms) >= REARM_LOW_MS) then
            armed = true
        end
    end

    ------------------------------------------------------------------
    -- Trigger-low debounce for aborting a running maneuver
    ------------------------------------------------------------------

    if running and trig_low then
        if not abort_low_since_ms then
            abort_low_since_ms = now_ms
        end
    else
        abort_low_since_ms = nil
    end

    local abort_requested =
        abort_low_since_ms and
        ((now_ms - abort_low_since_ms) >= ABORT_LOW_MS)

    ------------------------------------------------------------------
    -- Immediate mode safety
    ------------------------------------------------------------------

    if running and not in_allowed_mode(mode) then
        clear_overrides()

        running = false
        armed   = false

        abort_low_since_ms = nil

        gcs_msg("STOP: flight mode is not QHOVER or QLOITER")

        return update, 50
    end

    ------------------------------------------------------------------
    -- Trigger abort
    ------------------------------------------------------------------

    if running and abort_requested then
        clear_overrides()

        running = false
        armed   = false

        abort_low_since_ms = nil

        gcs_msg("STOP: script deactivated")

        return update, 50
    end

    ------------------------------------------------------------------
    -- Clear overrides while outside allowed modes
    ------------------------------------------------------------------

    if not running and not in_allowed_mode(mode) then
        clear_overrides()
        return update, 80
    end

    ------------------------------------------------------------------
    -- Start maneuver
    ------------------------------------------------------------------

    if not running and
       armed and
       trig_high and
       in_allowed_mode(mode) then

        axis_ch, axis_lbl = select_axis()

        local minimum, trim, maximum = rc_limits(axis_ch)
        local current_pwm = rcpwm(axis_ch)

        -- Start from the current pilot input when available.
        -- Otherwise, use the configured RC trim.
        if current_pwm > 0 then
            baseline_pwm = current_pwm
        else
            baseline_pwm = trim
        end

        target_pwm = clamp(
            baseline_pwm + input_sign() * AMP_US,
            minimum,
            maximum
        )

        t0_ms  = now_ms
        running = true
        armed   = false

        trigger_low_since_ms = nil
        abort_low_since_ms   = nil

        gcs_msg(string.format(
            "START %s step: RC%d baseline=%d target=%d sign=%d mode=%d",
            axis_lbl,
            axis_ch,
            baseline_pwm,
            target_pwm,
            input_sign(),
            mode
        ))

        return update, UPDATE_MS
    end

    ------------------------------------------------------------------
    -- Run maneuver
    ------------------------------------------------------------------

    if running then
        local elapsed_ms = now_ms - t0_ms

        if elapsed_ms >= T_STEP_MS then
            clear_overrides()

            running = false
            armed   = false

            gcs_msg("DONE")

            return update, 80
        end

        local output_pwm = target_pwm

        -- Optional linear fade-in.
        if FADE_IN_MS > 0 and elapsed_ms < FADE_IN_MS then
            local fade_fraction = elapsed_ms / FADE_IN_MS

            output_pwm =
                baseline_pwm +
                (target_pwm - baseline_pwm) * fade_fraction
        end

        set_rc_override(axis_ch, output_pwm)

        return update, UPDATE_MS
    end

    return update, 80
end

gcs_msg(
    string.format(
        "Initialized: QHOVER/QLOITER, ROLL/PITCH, sign=%d",
        input_sign()
    )
)

return update()