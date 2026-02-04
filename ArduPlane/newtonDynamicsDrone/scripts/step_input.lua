-- 1-step RC input override for QuadPlane (QSTABILIZE/QHOVER only)
-- CH9 HIGH  -> run one 1-step on selected axis (CH10 3-pos)
-- CH9 LOW   -> abort + clear overrides (debounced)
-- CH10 3-pos: LOW=ROLL, MID=PITCH, HIGH=YAW

local SCRIPT_NAME = "step_input"

-- ========= User configuration =========
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
local T_STEP_MS      = 900
local AMP_US         = 1000

-- IMPORTANT: set to 0 for a true step (no ramps)
local FADE_IN_MS     = 0

local UPDATE_MS      = 20

local MODE_QSTABILIZE = 17
local MODE_QHOVER     = 18

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

local function in_allowed_mode(mode)
    return (mode == MODE_QSTABILIZE) or (mode == MODE_QHOVER)
end

local function clear_overrides()
    if rc and rc.clear_overrides then rc:clear_overrides() end
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
    if running and (not in_allowed_mode(mode)) then
        clear_overrides()
        running = false
        gcs_msg("STOP (mode changed out of QSTABILIZE/QHOVER)")
        return update, 50
    end

    -- Start: only if ARMED and CH9 is HIGH
    if (not running) and armed and trig_high and in_allowed_mode(mode) then
        axis_ch, axis_lbl = select_axis()

        local mn, tr, mx = rc_limits(axis_ch)
        local cur = rcpwm(axis_ch)
        baseline_pwm = (cur > 0) and cur or tr

        -- constant target for the whole step (clamped)
        target_pwm = clamp(baseline_pwm + AMP_US, mn, mx)

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

        -- Abort only if CH9 is LOW continuously for ABORT_LOW_MS
        if low_since_ms and ((now_ms - low_since_ms) >= ABORT_LOW_MS) then
            clear_overrides()
            running = false
            gcs_msg("STOP")
            return update, 50
        end

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
