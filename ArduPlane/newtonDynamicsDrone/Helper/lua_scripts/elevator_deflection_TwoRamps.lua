---------------------------- DESCRIPTION --------------------------------
-- Elevator ramp: up → hold → down. Runs on CH9 HIGH, stops on CH9 LOW.
-- Amplitude selected by CH10 (3-pos). Re-armable any time.
-- MODE="RC" overrides elevator INPUT; MODE="SERVO" drives the servo output.

-------------------------------- CONFIG --------------------------------
local SCRIPT_NAME        = "elev_ramp"
local MODE               = "RC"        -- "RC" or "SERVO"
local ELEV_IN            = 2           -- elevator RC input channel (AETR usually 2)
local ELEV_SERVO         = 2           -- servo output number if MODE="SERVO" (set SERVO2_FUNCTION=94)

-- Trigger: 2-position switch (you can change these for a different controller)
local TRIG_CH            = 9           -- run/stop switch channel
local TRIG_HIGH_US       = 1800        -- ≥ triggers RUN
local TRIG_LOW_US        = 1200        -- ≤ triggers STOP

-- Amplitude: 3-position switch (you can change these for a different controller)
local AMP_CH             = 10          -- amplitude selector channel
local AMP_LOW_MAX_US     = 1300        -- ≤ LOW
local AMP_MID_MIN_US     = 1400        -- MID in [AMP_MID_MIN_US, AMP_MID_MAX_US]
local AMP_MID_MAX_US     = 1600
local AMP_HIGH_MIN_US    = 1700        -- ≥ HIGH

-- Ramp profile (adjustable)
local UP_TIME_MS         = 3000        -- ramp up duration
local HOLD_TIME_MS       = 3000        -- hold duration
local DOWN_TIME_MS       = 3000        -- ramp down duration
local NUM_PULSES         = 2           -- number of ramp pulses before auto-stop

-- Amplitudes relative to TRIM (PWM µs). Sign sets direction: + above TRIM, - below TRIM.
local AMP_SIGN           =  -1          -- +1 = towards higher PWM, -1 = lower PWM
local AMP_LOW_DELTA_US   =  80
local AMP_MID_DELTA_US   = 160
local AMP_HIGH_DELTA_US  = 240

-- Optional extra clamp. If nil, RC/servo limits are used as-is.
local RAMP_MIN_PWM       = nil         -- e.g., 1200
local RAMP_MAX_PWM       = nil         -- e.g., 1800
-------------------------------------------------------------------------

-- Severities and messaging
local SEV = { EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7 }
local function gcs_msg(sev, txt) gcs:send_text(sev, string.format("%s: %s", SCRIPT_NAME, txt)) end

-- Utils
local function now_ms() return millis() end
local function clamp(x, lo, hi) if x < lo then return lo elseif x > hi then return hi else return x end end

-- >>> IMPORTANT: always coerce API returns to plain Lua numbers <<<
local function pget(name, default)
    if not param or not param.get then return default end
    local v = param:get(name)
    if v == nil then return default end
    return tonumber(v) or default
end

local function rcpwm(ch)
    local v = rc:get_pwm(ch)
    if v == nil then return 0 end
    return tonumber(v) or 0
end

-- Limits (RC or SERVO), intersected with optional RAMP_MIN/MAX
local function get_limits()
    local mn, tr, mx
    if MODE == "RC" then
        mn = pget(string.format("RC%d_MIN",  ELEV_IN), 1100)
        tr = pget(string.format("RC%d_TRIM", ELEV_IN), 1500)
        mx = pget(string.format("RC%d_MAX",  ELEV_IN), 1900)
    else
        mn = pget(string.format("SERVO%d_MIN",  ELEV_SERVO), 1100)
        tr = pget(string.format("SERVO%d_TRIM", ELEV_SERVO), 1500)
        mx = pget(string.format("SERVO%d_MAX",  ELEV_SERVO), 1900)
    end
    if RAMP_MIN_PWM then mn = math.max(mn, tonumber(RAMP_MIN_PWM) or mn) end
    if RAMP_MAX_PWM then mx = math.min(mx, tonumber(RAMP_MAX_PWM) or mx) end
    return mn, tr, mx
end

-- IO adapters
local function set_elev_pwm(p)
    p = tonumber(p) or 0
    p = math.floor(p + 0.5)
    if MODE == "RC" then
        local ch = rc:get_channel(ELEV_IN)      -- reacquire every call
        if ch then ch:set_override(p) end
    else
        if SRV_Channels and SRV_Channels.set_output_pwm then
            SRV_Channels:set_output_pwm(ELEV_SERVO, p)
        end
    end
end

local function release_control(park_pwm)
    if MODE == "RC" then
        if rc and rc.clear_overrides then rc:clear_overrides() end
    else
        if park_pwm then set_elev_pwm(tonumber(park_pwm) or 1500) end
    end
end

-- Amplitude from CH10
local function select_delta()
    local a = rcpwm(AMP_CH)
    if a <= AMP_LOW_MAX_US then
        return AMP_LOW_DELTA_US
    elseif a >= AMP_HIGH_MIN_US then
        return AMP_HIGH_DELTA_US
    elseif a >= AMP_MID_MIN_US and a <= AMP_MID_MAX_US then
        return AMP_MID_DELTA_US
    else
        -- outside windows: default to MID
        return AMP_MID_DELTA_US
    end
end

-- Linear ramp
local function lerp(a, b, t_ms, T_ms)
    local u = (T_ms > 0) and clamp((tonumber(t_ms) or 0) / T_ms, 0, 1) or 1
    return a + (b - a) * u
end

-- State
local running      = false
local last_trig_hi = false
local phase        = "IDLE"   -- IDLE, UP, HOLD, DOWN
local phase_t0     = 0
local pulse_idx    = 0
local base_pwm     = 1500
local target_pwm   = 1600

-- Control
local function compute_targets()
    local mn, tr, mx = get_limits()
    local delta = (tonumber(select_delta()) or 0) * AMP_SIGN
    local tgt   = clamp(tr + delta, mn, mx)
    return mn, tr, mx, tgt
end

local function start_sequence()
    local mn, tr, mx, tgt = compute_targets()
    base_pwm   = clamp(rcpwm(ELEV_IN), mn, mx)  -- start from current input
    target_pwm = tgt
    pulse_idx  = 0
    phase      = "UP"
    phase_t0   = now_ms()
    running    = true
    gcs_msg(SEV.INFO, string.format("START base=%d target=%d mode=%s", base_pwm, target_pwm, MODE))
end

local function stop_sequence(reason)
    running = false
    phase   = "IDLE"
    release_control(base_pwm)
    gcs_msg(SEV.INFO, "STOP "..(reason or ""))
end

-- Main loop
function update()
    -- Trigger from CH9: HIGH runs, LOW stops
    local trig_pwm = rcpwm(TRIG_CH)
    local trig_hi  = trig_pwm >= TRIG_HIGH_US

    -- Rising edge → start; falling edge → stop
    if (not last_trig_hi) and trig_hi then
        start_sequence()
    elseif last_trig_hi and (trig_pwm <= TRIG_LOW_US) then
        stop_sequence("(trigger low)")
    end
    last_trig_hi = trig_hi

    if not running then
        return update, 50
    end

    local mn, tr, mx = get_limits()
    local tnow = now_ms()
    local dt   = tnow - phase_t0

    if phase == "UP" then
        local cmd = lerp(base_pwm, target_pwm, dt, UP_TIME_MS)
        set_elev_pwm(clamp(cmd, mn, mx))
        if dt >= UP_TIME_MS then
            phase   = "HOLD"; phase_t0 = tnow
        end
        return update, 20

    elseif phase == "HOLD" then
        set_elev_pwm(target_pwm)
        if dt >= HOLD_TIME_MS then
            phase   = "DOWN"; phase_t0 = tnow
        end
        return update, 20

    elseif phase == "DOWN" then
        local cmd = lerp(target_pwm, base_pwm, dt, DOWN_TIME_MS)
        set_elev_pwm(clamp(cmd, mn, mx))
        if dt >= DOWN_TIME_MS then
            pulse_idx = pulse_idx + 1
            gcs_msg(SEV.INFO, string.format("Pulse %d/%d done", pulse_idx, NUM_PULSES))
            if pulse_idx >= NUM_PULSES then
                stop_sequence("(completed)")
                return update, 50
            end
            -- If amplitude switch changed, use new target next pulse
            local _, _, _, tgt = compute_targets()
            target_pwm = tgt
            phase      = "UP"; phase_t0 = tnow
        end
        return update, 20
    end

    return update, 50
end

gcs_msg(SEV.INFO, "Initialized")
return update()