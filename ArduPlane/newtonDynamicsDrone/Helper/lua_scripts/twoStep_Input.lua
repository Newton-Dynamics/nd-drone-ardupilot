-- 2-step RC input override for QuadPlane (QSTABILIZE/QHOVER only)
-- CH9 HIGH  -> run one 2-step sequence on selected axis (from CH10 3-pos)
--              +step for T, then GAP, then -step for T
-- CH9 LOW   -> abort + clear overrides immediately
-- CH10 3-pos: LOW=ROLL, MID=PITCH, HIGH=YAW

local SCRIPT_NAME = "Two_step_input"

-- ========= User configuration =========
-- Activation switch (2-pos)
local TRIG_CH        = 9
local TRIG_HIGH_US   = 1800
local TRIG_LOW_US    = 1200

-- Axis selector (3-pos)
local SEL_CH         = 10
local SEL_LOW_MAX    = 1300          -- <= LOW  => ROLL
local SEL_MID_MIN    = 1400          -- MID in [MIN..MAX] => PITCH
local SEL_MID_MAX    = 1600
local SEL_HIGH_MIN   = 1700          -- >= HIGH => YAW

-- RC input channels (AETR typical for Plane: Roll=1, Pitch=2, Throttle=3, Yaw=4)
local ROLL_IN        = 1
local PITCH_IN       = 2
local YAW_IN         = 4

-- timing/amplitude
local T_STEP_MS      = 900           -- base step duration T
local AMP_US         = 350           -- amplitude in microseconds (start small: 30..80)
local FADE_IN_MS     = 300           -- fade-in time at start of EACH step pulse

-- GAP between + and - step (seconds)
local STPI_GAP_S     = 4.0           -- 7 seconds "nothing" between +AMP and -AMP

-- Update period
local UPDATE_MS      = 20            -- 50Hz

-- Only allow in these ArduPlane QuadPlane modes:
local MODE_QSTABILIZE = 17
local MODE_QHOVER     = 18

-- Optional clamp (leave nil to use RCx_MIN/MAX)
local CMD_MIN_PWM    = nil
local CMD_MAX_PWM    = nil
-- =====================================

-- Messaging
local SEV_INFO = 6
local function gcs_msg(txt)
    gcs:send_text(SEV_INFO, string.format("%s: %s", SCRIPT_NAME, txt))
end

-- Helpers
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

-- RC limits/trim for a given input channel, intersect with optional CMD_MIN/MAX
local function rc_limits(ch_in)
    local mn = pget(string.format("RC%d_MIN",  ch_in), 1000)
    local tr = pget(string.format("RC%d_TRIM", ch_in), 1500)
    local mx = pget(string.format("RC%d_MAX",  ch_in), 2000)
    if CMD_MIN_PWM then mn = math.max(mn, tonumber(CMD_MIN_PWM) or mn) end
    if CMD_MAX_PWM then mx = math.min(mx, tonumber(CMD_MAX_PWM) or mx) end
    return mn, tr, mx
end

-- CH10 -> axis selection and label
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

-- Apply RC override to chosen input channel
local function set_rc_override(ch_in, pwm)
    local ch = rc:get_channel(ch_in)
    if ch then
        ch:set_override(math.floor((tonumber(pwm) or 0) + 0.5))
    end
end

-- Config gap (seconds) -> ms
local function gap_ms_from_cfg()
    local s = tonumber(STPI_GAP_S) or 7.0
    if s < 0 then s = 0 end
    return math.floor(s * 1000 + 0.5)
end

-- 2-step segment definition:
-- +A for 1T, then 0 for GAP, then -A for 1T
-- Returns: sign, t_in_this_segment_ms (for per-step fade-in)
local function seg_for_time(t_ms, gap_ms)
    local T = T_STEP_MS
    local G = gap_ms or 0

    if t_ms < T then
        return  1, t_ms
    elseif t_ms < (T + G) then
        return  0, 0
    elseif t_ms < (T + G + T) then
        return -1, (t_ms - (T + G))
    else
        return  0, 0
    end
end

local function total_duration_ms(gap_ms)
    return (2 * T_STEP_MS) + (gap_ms or 0)
end

-- State
local running      = false
local last_mode    = nil
local last_trig    = false
local t0_ms        = 0
local axis_ch      = nil
local axis_lbl     = ""
local baseline_pwm = nil
local gap_ms_run   = 7000   -- captured at start of sequence

function update()
    local mode = vehicle:get_mode() or -1
    local trig = rcpwm(TRIG_CH)

    local trig_on  = trig >= TRIG_HIGH_US
    local trig_off = trig <= TRIG_LOW_US

    -- rising edge detection: start only once per toggle
    local rising = trig_on and (not last_trig)
    last_trig = trig_on

    -- Safety: if we leave Q modes while running, stop immediately
    if running and (not in_allowed_mode(mode)) then
        clear_overrides()
        running = false
        gcs_msg("STOP (mode changed out of QSTABILIZE/QHOVER)")
        last_mode = mode
        return update, 50
    end

    -- CH9 LOW always releases
    if trig_off and running then
        clear_overrides()
        running = false
        gcs_msg("STOP (released)")
        last_mode = mode
        return update, 50
    end

    -- Start only on rising edge, and only in allowed modes
    if rising and in_allowed_mode(mode) then
        axis_ch, axis_lbl = select_axis()
        local _, tr, _ = rc_limits(axis_ch)
        local cur = rcpwm(axis_ch)
        baseline_pwm = (cur > 0) and cur or tr

        gap_ms_run = gap_ms_from_cfg()

        t0_ms = millis()
        running = true
        gcs_msg(string.format(
            "START 2-step on %s (RC%d baseline=%d, GAP=%.2fs)",
            axis_lbl, axis_ch, baseline_pwm, gap_ms_run / 1000.0
        ))
        last_mode = mode
        return update, UPDATE_MS
    end

    -- If trigger held high but not in Q modes, ensure no overrides remain
    if trig_on and (not in_allowed_mode(mode)) and running then
        clear_overrides()
        running = false
        gcs_msg("STOP (ignored: not in QSTABILIZE/QHOVER)")
        last_mode = mode
        return update, 80
    end

    -- Run sequence
    if running then
        local t_ms = (millis() - t0_ms)
        if t_ms >= total_duration_ms(gap_ms_run) then
            clear_overrides()
            running = false
            gcs_msg("DONE (sequence complete)")
            last_mode = mode
            return update, 80
        end

        local sign, t_seg = seg_for_time(t_ms, gap_ms_run)
        local delta = sign * AMP_US

        -- fade in at start of EACH non-zero step pulse
        if sign ~= 0 and FADE_IN_MS > 0 and t_seg < FADE_IN_MS then
            delta = delta * (t_seg / FADE_IN_MS)
        end

        local mn, _, mx = rc_limits(axis_ch)
        local target = clamp(baseline_pwm + delta, mn, mx)

        -- Apply override only to the selected axis; clear others to avoid sticking
        if axis_ch ~= ROLL_IN  then set_rc_override(ROLL_IN,  rcpwm(ROLL_IN))  end
        if axis_ch ~= PITCH_IN then set_rc_override(PITCH_IN, rcpwm(PITCH_IN)) end
        if axis_ch ~= YAW_IN   then set_rc_override(YAW_IN,   rcpwm(YAW_IN))   end

        set_rc_override(axis_ch, target)
        last_mode = mode
        return update, UPDATE_MS
    end

    last_mode = mode
    return update, 80
end

gcs_msg("Initialized (CH9=run, CH10=axis; QSTABILIZE/QHOVER only; STPI_GAP_S controls gap)")
return update()
