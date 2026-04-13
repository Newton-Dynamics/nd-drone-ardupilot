-- Elevator staircase with 3-position trigger (MID=off, LOW=down, HIGH=up)
-- Auto-stops at RC2_MAX / RC2_MIN and releases control

------------------------------ CONFIG ------------------------------
local MODE       = "RC"   -- "RC" override input; or "SERVO" drive output
local ELEV_IN    = 13      -- elevator RC input channel (AETR usually 2)
local TRIG_CH    = 10     -- 3-position trigger channel

-- 3-pos windows (adjust if needed)
local LOW_MAX    = 1300   -- <= LOW_MAX  -> LOW
local MID_MIN    = 1400   -- MID in [MID_MIN, MID_MAX]
local MID_MAX    = 1600
local HIGH_MIN   = 1700   -- >= HIGH_MIN -> HIGH

-- step profile
local STEP_US    = 90
local HOLD_MS    = 3000

-- SERVO mode only:
local ELEV_SERVO = 2      -- set SERVO2_FUNCTION=94 (Scripting) if MODE="SERVO"
-------------------------------------------------------------------

-- state
local running   = false
local dir       = 0       -- +1 = down (PWM up), -1 = up (PWM down), 0 = idle
local base_pwm  = 1500
local cmd_pwm   = 1500
local next_t    = 0
local last_state = "MID"

-- utils
local function now() return millis() end
local function clamp(x,lo,hi) if x<lo then return lo elseif x>hi then return hi else return x end end
local function rcpwm(ch) return rc:get_pwm(ch) or 0 end
local function state_from_pwm(p)
    if p <= LOW_MAX then return "LOW"
    elseif p >= HIGH_MIN then return "HIGH"
    elseif p >= MID_MIN and p <= MID_MAX then return "MID"
    else return last_state end
end

-- param helpers
local function pget(name, default)
    if not param or not param.get then return default end
    local v = param:get(name); if v == nil then return default end
    return v
end

local function read_limits()
    if MODE == "RC" then
        local mn = pget(string.format("RC%d_MIN",  ELEV_IN), 1100)
        local tr = pget(string.format("RC%d_TRIM", ELEV_IN), 1500)
        local mx = pget(string.format("RC%d_MAX",  ELEV_IN), 1900)
        return mn, tr, mx
    else
        local mn = pget(string.format("SERVO%d_MIN",  ELEV_SERVO), 1100)
        local tr = pget(string.format("SERVO%d_TRIM", ELEV_SERVO), 1500)
        local mx = pget(string.format("SERVO%d_MAX",  ELEV_SERVO), 1900)
        return mn, tr, mx
    end
end

-- IO
local function set_elev_pwm(p)
    p = math.floor(p + 0.5)
    if MODE == "RC" then
        local ch = rc:get_channel(ELEV_IN)
        if ch then ch:set_override(p) end
    else
        if SRV_Channels and SRV_Channels.set_output_pwm then
            SRV_Channels:set_output_pwm(ELEV_SERVO, p)
        end
    end
end

local function clear_outputs()
    if MODE == "RC" then
        if rc and rc.clear_overrides then rc:clear_overrides() end
    else
        set_elev_pwm(base_pwm)  -- SERVO mode parks at baseline; controller cannot take over
    end
end

-- control
local function start(direction)
    local _, tr, _ = read_limits()
    base_pwm = tr
    cmd_pwm  = base_pwm
    dir      = direction
    running  = true
    next_t   = now()
    gcs:send_text(6, string.format("ElevStep: START dir=%d base=%d mode=%s", dir, base_pwm, MODE))
end

local function stop(reason)
    running = false
    dir     = 0
    clear_outputs()          -- releases control to pilot in RC mode
    gcs:send_text(6, "ElevStep: STOP "..(reason or ""))
end

-- main loop
function update()
    local trig = rcpwm(TRIG_CH)
    local st = state_from_pwm(trig)

    if st ~= last_state then
        if st == "MID" then
            stop("(MID)")
        elseif st == "LOW" then
            start(1)          -- TRIM → MAX
        elseif st == "HIGH" then
            start(-1)         -- TRIM → MIN
        end
        last_state = st
    end

    if not running then
        return update, 50
    end

    local mn, _, mx = read_limits()
    local t = now()
    if t < next_t then return update, 20 end

    -- compute next step
    local next_cmd = cmd_pwm + dir * STEP_US

    -- auto-stop at limits (command the limit once, then release)
    if dir > 0 and next_cmd >= mx then
        cmd_pwm = mx
        set_elev_pwm(cmd_pwm)
        gcs:send_text(6, string.format("ElevStep: cmd=%d (MAX)", cmd_pwm))
        stop("(reached MAX)")
        return update, 50
    elseif dir < 0 and next_cmd <= mn then
        cmd_pwm = mn
        set_elev_pwm(cmd_pwm)
        gcs:send_text(6, string.format("ElevStep: cmd=%d (MIN)", cmd_pwm))
        stop("(reached MIN)")
        return update, 50
    end

    -- regular step
    cmd_pwm = clamp(next_cmd, mn, mx)
    set_elev_pwm(cmd_pwm)
    gcs:send_text(6, string.format("ElevStep: cmd=%d", cmd_pwm))

    next_t = t + HOLD_MS
    return update, 20
end

gcs:send_text(6, "ElevStep: loaded (3-pos, auto-stop at limits)")
return update()
