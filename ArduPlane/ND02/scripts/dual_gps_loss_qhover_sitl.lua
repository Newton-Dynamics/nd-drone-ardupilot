-- Dual GPS loss failsafe for ArduPlane 4.6.3 QuadPlane
-- SITL deployment label. GPS detection is backend-independent.
--
-- Intended action:
--   QLOITER, or AUTO while currently in VTOL operation
--   + armed + likely flying
--   + GPS1 and GPS2 both unusable for ALL_GPS_LOSS_DEBOUNCE_MS
--       => switch to QHOVER
--
-- Install only one copy of this script.

local SCRIPT_NAME = "GPSFS"
local DEPLOYMENT_NAME = "SITL-uBlox"

-- ---------------- User settings ----------------
local UPDATE_PERIOD_MS             = 100   -- 10 Hz script update
local GPS_MESSAGE_TIMEOUT_MS       = 2000  -- latest GPS message must be newer
local GPS_FIX_TIMEOUT_MS           = 2000  -- latest valid fix must be newer
local GPS_FAIL_REPORT_DELAY_MS     = 500   -- debounce individual failure messages
local GPS_RECOVERY_CONFIRM_MS      = 1000  -- recovered GPS must remain usable
local ALL_GPS_LOSS_DEBOUNCE_MS     = 3000  -- both GPS must remain unavailable
local MODE_CHANGE_RETRY_MS         = 1000

-- ArduPlane 4.6.3 mode numbers
local MODE_AUTO    = 10
local MODE_QHOVER  = 18
local MODE_QLOITER = 19

-- MAV_SEVERITY values
local SEVERITY_CRITICAL = 2
local SEVERITY_ERROR    = 3
local SEVERITY_WARNING  = 4
local SEVERITY_NOTICE   = 5
local SEVERITY_INFO     = 6

local sensor_state = {
    { failed_reported = false, fail_since = nil, recovery_since = nil },
    { failed_reported = false, fail_since = nil, recovery_since = nil },
}

local all_loss_since = nil
local any_recovery_since = nil
local last_mode_attempt_ms = nil
local failsafe_latched = false
local was_armed = false

local function elapsed_ms(now, then_ms)
    return (now - then_ms):tofloat()
end

local function send_text(severity, text)
    gcs:send_text(severity, SCRIPT_NAME .. ": " .. text)
end

-- Returns:
--   usable: true only with 3D-or-better fix and fresh data
--   reason: concise reason when unusable
local function gps_usable(instance, now)
    local count = gps:num_sensors()
    if count <= instance then
        return false, "not detected"
    end

    local status = gps:status(instance)
    if status < gps.GPS_OK_FIX_3D then
        return false, string.format("fix=%d", status)
    end

    local last_message = gps:last_message_time_ms(instance)
    local message_age = elapsed_ms(now, last_message)
    if message_age > GPS_MESSAGE_TIMEOUT_MS then
        return false, string.format("msg stale %.1fs", message_age * 0.001)
    end

    local last_fix = gps:last_fix_time_ms(instance)
    local fix_age = elapsed_ms(now, last_fix)
    if fix_age > GPS_FIX_TIMEOUT_MS then
        return false, string.format("fix stale %.1fs", fix_age * 0.001)
    end

    return true, "healthy"
end

local function update_sensor_message(instance, usable, reason, now)
    local state = sensor_state[instance + 1]
    local label = "GPS" .. tostring(instance + 1)

    if usable then
        state.fail_since = nil

        if state.failed_reported then
            if state.recovery_since == nil then
                state.recovery_since = now
            elseif elapsed_ms(now, state.recovery_since) >= GPS_RECOVERY_CONFIRM_MS then
                state.failed_reported = false
                state.recovery_since = nil
                send_text(SEVERITY_NOTICE, label .. " recovered")
            end
        else
            state.recovery_since = nil
        end
        return
    end

    state.recovery_since = nil

    if not state.failed_reported then
        if state.fail_since == nil then
            state.fail_since = now
        elseif elapsed_ms(now, state.fail_since) >= GPS_FAIL_REPORT_DELAY_MS then
            state.failed_reported = true
            state.fail_since = nil
            send_text(SEVERITY_WARNING, label .. " failed: " .. reason)
        end
    end
end

local function in_target_operation()
    local mode = vehicle:get_mode()

    if mode == MODE_QLOITER then
        return true
    end

    -- AUTO is targeted only while the QuadPlane is actually in VTOL operation.
    if mode == MODE_AUTO and quadplane:in_vtol_mode() then
        return true
    end

    return false
end

local function reset_pending_loss()
    all_loss_since = nil
    any_recovery_since = nil
    last_mode_attempt_ms = nil
end

local function update()
    local now = millis()
    local armed = arming:is_armed()

    -- Re-arm starts a new failsafe cycle. A triggered event remains latched
    -- for the remainder of the current armed flight.
    if not armed then
        if was_armed and failsafe_latched then
            send_text(SEVERITY_INFO, "disarmed; failsafe latch reset")
        end
        failsafe_latched = false
        reset_pending_loss()
    end
    was_armed = armed

    local gps1_ok, gps1_reason = gps_usable(0, now)
    local gps2_ok, gps2_reason = gps_usable(1, now)

    update_sensor_message(0, gps1_ok, gps1_reason, now)
    update_sensor_message(1, gps2_ok, gps2_reason, now)

    if failsafe_latched then
        return update, UPDATE_PERIOD_MS
    end

    local eligible = armed and vehicle:get_likely_flying() and in_target_operation()

    if not eligible then
        reset_pending_loss()
        return update, UPDATE_PERIOD_MS
    end

    local both_unusable = (not gps1_ok) and (not gps2_ok)

    if both_unusable then
        any_recovery_since = nil

        if all_loss_since == nil then
            all_loss_since = now
            send_text(SEVERITY_CRITICAL, "both GPS lost; debounce started")
        end

        if elapsed_ms(now, all_loss_since) >= ALL_GPS_LOSS_DEBOUNCE_MS then
            local retry_allowed = last_mode_attempt_ms == nil or
                elapsed_ms(now, last_mode_attempt_ms) >= MODE_CHANGE_RETRY_MS

            if retry_allowed then
                last_mode_attempt_ms = now
                local changed = vehicle:set_mode(MODE_QHOVER)

                if changed then
                    failsafe_latched = true
                    send_text(SEVERITY_CRITICAL, "QHOVER triggered: both GPS lost")
                else
                    send_text(SEVERITY_ERROR, "QHOVER mode change failed; retrying")
                end
            end
        end

        return update, UPDATE_PERIOD_MS
    end

    -- At least one GPS looks usable. Do not cancel a pending all-GPS-loss
    -- event until that recovery remains continuously healthy for the
    -- configured confirmation period.
    if all_loss_since ~= nil then
        if any_recovery_since == nil then
            any_recovery_since = now
        elseif elapsed_ms(now, any_recovery_since) >= GPS_RECOVERY_CONFIRM_MS then
            send_text(SEVERITY_NOTICE, "GPS recovered; QHOVER cancelled")
            reset_pending_loss()
        end
    end

    return update, UPDATE_PERIOD_MS
end

send_text(SEVERITY_INFO, "initialized (" .. DEPLOYMENT_NAME .. ")")
return update, UPDATE_PERIOD_MS
