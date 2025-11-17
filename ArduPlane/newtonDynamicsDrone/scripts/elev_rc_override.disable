-- RC override of elevator input (AETR). Timing-free.
-- CH9 HIGH -> apply TRIM ± delta on RC<ELEV_IN>; CH9 LOW -> clear overrides (pilot regains control).
-- CH10 (3-pos) selects amplitude; CH7 (2-pos) selects direction.

------------------------------- CONFIG --------------------------------
local SCRIPT_NAME        = "elev_rcelev_override"
local ELEV_IN            = 2           -- elevator RC input channel (AETR usually 2)

-- Run/stop trigger (CH9, 2-pos)
local TRIG_CH            = 9
local TRIG_HIGH_US       = 1800        -- ≥ RUN
local TRIG_LOW_US        = 1200        -- ≤ STOP

-- Direction selector (CH7, 2-pos)
local DIR_CH             = 7
local DIR_LOW_MAX_US     = 1300        -- CH7 ≤ this => DOWN (PWM up)
local DIR_HIGH_MIN_US    = 1700        -- CH7 ≥ this => UP   (PWM down)

-- Amplitude selector (CH10, 3-pos)
local AMP_CH             = 10
local AMP_LOW_MAX_US     = 1300        -- ≤ LOW
local AMP_MID_MIN_US     = 1400        -- MID in [MIN..MAX]
local AMP_MID_MAX_US     = 1600
local AMP_HIGH_MIN_US    = 1700        -- ≥ HIGH

-- Amplitude magnitudes (µs around RC trim). Tweak for more/less deflection.
local AMP_LOW_DELTA_US   =  80
local AMP_MID_DELTA_US   = 160
local AMP_HIGH_DELTA_US  = 240

-- Optional extra clamp on commanded PWM (leave nil to use RCx_MIN/MAX)
local CMD_MIN_PWM        = nil         -- e.g., 1200
local CMD_MAX_PWM        = nil         -- e.g., 1800
------------------------------------------------------------------------

-- Messaging
local SEV_INFO = 6
local function gcs_msg(txt) gcs:send_text(SEV_INFO, string.format("%s: %s", SCRIPT_NAME, txt)) end

-- Helpers
local function clamp(x, lo, hi) if x < lo then return lo elseif x > hi then return hi else return x end end
local function pget(name, default)
    if not param or not param.get then return default end
    local v = param:get(name); return tonumber(v) or default
end
local function rcpwm(ch) local v = rc:get_pwm(ch); return tonumber(v) or 0 end

-- RC limits/trim for ELEV_IN, intersect with optional CMD_MIN/MAX
local function rc_limits()
    local mn = pget(string.format("RC%d_MIN",  ELEV_IN), 1100)
    local tr = pget(string.format("RC%d_TRIM", ELEV_IN), 1500)
    local mx = pget(string.format("RC%d_MAX",  ELEV_IN), 1900)
    if CMD_MIN_PWM then mn = math.max(mn, tonumber(CMD_MIN_PWM) or mn) end
    if CMD_MAX_PWM then mx = math.min(mx, tonumber(CMD_MAX_PWM) or mx) end
    return mn, tr, mx
end

-- CH10 → delta magnitude and label
local function select_delta()
    local a = rcpwm(AMP_CH)
    if a <= AMP_LOW_MAX_US then return AMP_LOW_DELTA_US, "LOW"
    elseif a >= AMP_HIGH_MIN_US then return AMP_HIGH_DELTA_US, "HIGH"
    elseif a >= AMP_MID_MIN_US and a <= AMP_MID_MAX_US then return AMP_MID_DELTA_US, "MID"
    else return AMP_MID_DELTA_US, "MID" end
end

-- CH7 → direction sign (+1 down/PWM↑, −1 up/PWM↓) with simple hysteresis
local last_sign = 1
local function select_dir_sign()
    local d = rcpwm(DIR_CH)
    if d <= DIR_LOW_MAX_US    then last_sign =  1
    elseif d >= DIR_HIGH_MIN_US then last_sign = -1
    end
    return last_sign
end

-- Apply RC override to elevator input
local function set_elev_rc(pwm)
    local ch = rc:get_channel(ELEV_IN)
    if ch then ch:set_override(math.floor((tonumber(pwm) or 0) + 0.5)) end
end

-- Clear all RC overrides
local function clear_overrides()
    if rc and rc.clear_overrides then rc:clear_overrides() end
end

-- State for log de-spam
local running, last_cmd, last_amp_lbl, last_dir_lbl = false, nil, "", ""

function update()
    local trig = rcpwm(TRIG_CH)
    local run  = trig >= TRIG_HIGH_US
    local stop = trig <= TRIG_LOW_US

    if run then
        local mn, tr, mx = rc_limits()
        local delta, amp_lbl = select_delta()
        local sign = select_dir_sign()
        local target = clamp(tr + sign * delta, mn, mx)
        set_elev_rc(target)

        local dir_lbl = (sign > 0) and "DOWN" or "UP"
        if (not running) or (last_cmd ~= target) or (amp_lbl ~= last_amp_lbl) or (dir_lbl ~= last_dir_lbl) then
            gcs_msg(string.format("RUN dir=%s amp=%s target=%d", dir_lbl, amp_lbl, target))
        end
        running       = true
        last_cmd      = target
        last_amp_lbl  = amp_lbl
        last_dir_lbl  = dir_lbl
        return update, 30
    end

    if stop and running then
        clear_overrides()               -- instant pilot takeover
        gcs_msg("STOP (released)")
        running, last_cmd = false, nil
        return update, 60
    end

    return update, 80
end

gcs_msg("Initialized")
return update()
