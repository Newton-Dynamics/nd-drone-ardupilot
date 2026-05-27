-- Author: Hussein Sleiman
-- Release: February 19, 2026

-- ==================================== Description ===================================================
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

-- ========= User configuration =========
local SEL_HIGH_MIN   = 1700

local DIR_CH           = 6      
local DIR_PLUS_US      = 1800
local DIR_MINUS_US     = 1200   
local DIR_DEFAULT_PLUS = true   

local TRIG_CH        = 9
local TRIG_HIGH_US   = 1800
local TRIG_LOW_US    = 1200

local SEL_CH         = 10
local SEL_LOW_MAX    = 1300
local SEL_MID_MIN    = 1400
local SEL_MID_MAX    = 1600
local SEL_HIGH_MIN   = 1700

-- RC input channels (AETR typical for Plane: Roll=1, Pitch=2, Throttle=3, Yaw=4)
local ROLL_IN        = 1
local PITCH_IN       = 2
local YAW_IN         = 4

-- Step definition
local T_STEP_MS      = 1000
local AMP_US         = 1000

-- IMPORTANT: set to 0 for a true step (no ramps)
local FADE_IN_MS     = 0

local UPDATE_MS      = 20

--local modes allowed to use the script
local MODE_QSTABILIZE = 17
local MODE_QHOVER = 18
local MODE_QLOITER = 19

-- Debounce / arm logic (prevents multiple pulses)
local REARM_LOW_MS   = 300   -- must be LOW this long before a new start is allowed
local ABORT_LOW_MS   = 150   -- must be LOW this long to abort during a run
-- =====================================

local SEV_INFO = 6
local function gcs_msg(txt)
    gcs:send_text(SEV_INFO, string.format("%s: %s", SCRIPT_NAME, txt))
end

local function clamp(x, lo, hi)
    if x < lo then return lo elseif x > hi then return hi else return x end
end

local function pget(name, default)
    local v = param:get(name)
    return tonumber(v) or default
end

local function rcpwm(ch)
    local v = rc:get_pwm(ch)
    return tonumber(v) or 0
end

local function step_sign()
    local v = rcpwm(DIR_CH)
    if v == 0 then
        return DIR_DEFAULT_PLUS and 1 or -1
    end
    if v >= DIR_PLUS_US then
        return 1
    elseif v <= DIR_MINUS_US then
        return -1
    else
        -- switch is in-between thresholds; use default
        return DIR_DEFAULT_PLUS and 1 or -1
    end
end

local function in_allowed_mode(mode)
    return (mode == MODE_QSTABILIZE) or (mode == MODE_QHOVER) or (mode == MODE_QLOITER)
end

local function clear_overrides()
    -- Explicitly clear Lua RC_Channel overrides (robust across firmwares)
    local chans = { ROLL_IN, PITCH_IN, YAW_IN }
    for i = 1, #chans do
        local ch = rc:get_channel(chans[i])
        if ch then
            if ch.clear_override then
                ch:clear_override()
            else
                -- fallback used on some older firmwares
                ch:set_override(0)
            end
        end
    end

    -- keep this as an extra cleanup (harmless if supported)
    if rc and rc.clear_overrides then
        rc:clear_overrides()
    end
end

local function rc_limits(ch_in)
    local mn = pget(string.format("RC%d_MIN",  ch_in), 1000)
    local tr = pget(string.format("RC%d_TRIM", ch_in), 1500)
    local mx = pget(string.format("RC%d_MAX",  ch_in), 2000)
    return mn, tr, mx
end

local function select_axis()
    local s = rcpwm(SEL_CH)
    if s <= SEL_LOW_MAX then
        return ROLL_IN, "ROLL"
    elseif s >= SEL_HIGH_MIN then
        return YAW_IN, "YAW"
    elseif s >= SEL_MID_MIN and s <= SEL_MID_MAX then
        return PITCH_IN, "PITCH"
    else
        return PITCH_IN, "PITCH"
    end
end

local function set_rc_override(ch_in, pwm)
    local ch = rc:get_channel(ch_in)
    if ch then
        ch:set_override(math.floor((tonumber(pwm) or 0) + 0.5))
    end
end

-- State
local running        = false
local armed          = false
local low_since_ms   = nil

local t0_ms          = 0
local axis_ch        = nil
local axis_lbl       = ""
local baseline_pwm   = 1500
local target_pwm     = 1500

function update()
    local now_ms = millis()
    local mode   = vehicle:get_mode() or -1
    local trig   = rcpwm(TRIG_CH)

    local trig_low  = (trig <= TRIG_LOW_US)
    local trig_high = (trig >= TRIG_HIGH_US)

    -- Track how long CH9 has been continuously LOW
    if trig_low then
        if not low_since_ms then low_since_ms = now_ms end
    else
        low_since_ms = nil
    end

    -- Arm only after a stable LOW period (prevents re-triggers/glitches)
    if (not running) then
        if low_since_ms and ((now_ms - low_since_ms) >= REARM_LOW_MS) then
            armed = true
        end
    end

    -- Safety: leaving Q modes stops immediately
    if running and (trig_low or (not in_allowed_mode(mode))) then
        clear_overrides()
        running = false
        if trig_low then
            gcs_msg("STOP (script deactivated)")
        else
            gcs_msg("STOP (mode not correct)")
        end
        return update, 50
    end
    
    if (not running) and (not in_allowed_mode(mode)) then
        clear_overrides()
        return update, 80
    end

    -- Start: only if ARMED and CH9 is HIGH
    if (not running) and armed and trig_high and in_allowed_mode(mode) then
        axis_ch, axis_lbl = select_axis()

        local mn, tr, mx = rc_limits(axis_ch)
        local cur = rcpwm(axis_ch)
        baseline_pwm = (cur > 0) and cur or tr

        -- constant target for the whole step (clamped)
        local sgn = step_sign()  -- +1 or -1 from DIR_CH
        target_pwm = clamp(baseline_pwm + sgn * AMP_US, mn, mx)

        t0_ms = now_ms
        running = true
        armed = false

        gcs_msg(string.format(
            "START step on %s (RC%d baseline=%d target=%d)",
            axis_lbl, axis_ch, baseline_pwm, target_pwm
        ))
        return update, UPDATE_MS
    end

    -- Run
    if running then
        local t_ms = now_ms - t0_ms
        -- Done
        if t_ms >= T_STEP_MS then
            clear_overrides()
            running = false
            gcs_msg("DONE")
            return update, 80
        end

        -- Perfect step (or optional fade-in)
        local out = target_pwm
        if FADE_IN_MS > 0 and t_ms < FADE_IN_MS then
            local a = t_ms / FADE_IN_MS
            out = baseline_pwm + (target_pwm - baseline_pwm) * a
        end

        set_rc_override(axis_ch, out)
        return update, UPDATE_MS
    end

    return update, 80
end

gcs_msg("Step-Input init")
return update()
