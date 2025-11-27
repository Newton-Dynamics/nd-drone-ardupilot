-- Throttle RC override script for 3-position input (FBWA-only).
-- CH9 HIGH  -> set throttle RC input to a preset from CH10
-- CH9 LOW   -> clear overrides (pilot regains control instantly)
-- Edit THR_*_PWM to change the three throttle levels.
-- Edit TRIG_CH and AMP_CH to change the input channels if different remote mapping is used.
------------------------------------------------------------------------

local SCRIPT_NAME          = "thr_rc_override_fbwa"

-- Throttle RC input channel (AETR usually 3)
local THR_IN               = 3

-- Require this flight mode to allow override (ArduPlane: FBWA = 5)
local MODE_REQUIRED        = 5         -- FBWA

-- Run/stop trigger (CH9, 2-pos)
local TRIG_CH              = 9
local TRIG_HIGH_US         = 1800      -- ≥ RUN
local TRIG_LOW_US          = 1200      -- ≤ STOP

-- Level selector (CH10, 3-pos)
local AMP_CH               = 10
local AMP_LOW_MAX_US       = 1300      -- ≤ LOW
local AMP_MID_MIN_US       = 1400      -- MID in [MIN..MAX]
local AMP_MID_MAX_US       = 1600
local AMP_HIGH_MIN_US      = 1700      -- ≥ HIGH

-- Throttle presets (raw PWM). Adjust here.
local THR_LOW_PWM          = 1300      -- "LOW"
local THR_MID_PWM          = 1500      -- "MID"
local THR_HIGH_PWM         = 1700      -- "HIGH"

-- Optional extra clamp of commanded PWM (leave nil to use RCx_MIN/MAX)
local CMD_MIN_PWM          = nil       -- e.g., 1200
local CMD_MAX_PWM          = nil       -- e.g., 1900
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

-- RC limits/trim for throttle, intersect with optional CMD_MIN/MAX
local function rc_limits()
    local mn = pget(string.format("RC%d_MIN",  THR_IN), 1000)
    local tr = pget(string.format("RC%d_TRIM", THR_IN), 1500)
    local mx = pget(string.format("RC%d_MAX",  THR_IN), 2000)
    if CMD_MIN_PWM then mn = math.max(mn, tonumber(CMD_MIN_PWM) or mn) end
    if CMD_MAX_PWM then mx = math.min(mx, tonumber(CMD_MAX_PWM) or mx) end
    return mn, tr, mx
end

-- CH10 -> preset PWM and label
local function select_target_pwm()
    local a = rcpwm(AMP_CH)
    if a <= AMP_LOW_MAX_US then 
        return THR_LOW_PWM, "LOW"

    elseif a >= AMP_HIGH_MIN_US then 
        return THR_HIGH_PWM, "HIGH"

    elseif a >= AMP_MID_MIN_US and a <= AMP_MID_MAX_US then 
        return THR_MID_PWM, "MID"

    else return THR_MID_PWM, "MID" 
        
    end
end

-- Apply RC override to throttle input
local function set_thr_rc(pwm)
    local ch = rc:get_channel(THR_IN)
    if ch then ch:set_override(math.floor((tonumber(pwm) or 0) + 0.5)) end
end

-- Clear RC overrides
local function clear_overrides()
    if rc and rc.clear_overrides then rc:clear_overrides() end
end

-- State
local running, last_cmd, last_lbl = false, nil, ""
local last_mode = nil

function update()
    local mode = vehicle:get_mode() or -1
    local trig = rcpwm(TRIG_CH)
    local run  = trig >= TRIG_HIGH_US
    local stop = trig <= TRIG_LOW_US

    -- If we were running and we left FBWA, immediately release
    if running and (mode ~= MODE_REQUIRED) then
        clear_overrides()
        running, last_cmd = false, nil
        gcs_msg("STOP (mode changed out of FBWA)")
        -- fall through; no return so we can also process CH9 LOW edge
    end

    -- Only allow activation when in FBWA
    if run and (mode == MODE_REQUIRED) then
        local mn, _, mx = rc_limits()
        local raw, lbl = select_target_pwm()
        local target = clamp(raw, mn, mx)
        set_thr_rc(target)

        if (not running) or (last_cmd ~= target) or (lbl ~= last_lbl) or (mode ~= last_mode) then
            gcs_msg(string.format("RUN %s -> %d (FBWA only; clamp [%d..%d])", lbl, target, mn, mx))
        end
        running   = true
        last_cmd  = target
        last_lbl  = lbl
        last_mode = mode
        return update, 30
    end

    -- CH9 LOW always releases, regardless of mode
    if stop and running then
        clear_overrides()
        running, last_cmd = false, nil
        gcs_msg("STOP (released)")
        last_mode = mode
        return update, 60
    end

    -- If CH9 HIGH but not FBWA, ensure we are not overriding
    if run and (mode ~= MODE_REQUIRED) and running then
        clear_overrides()
        running, last_cmd = false, nil
        gcs_msg("STOP (ignored: not FBWA)")
        return update, 80
    end

    last_mode = mode
    return update, 80
end

gcs_msg("Initialized")
return update()